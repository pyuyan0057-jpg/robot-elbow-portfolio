"""Closed-loop ROS 2 transport for seven joint targets, encoders and tendon feedback."""

import os, sys, json, time, queue, threading, subprocess, shutil
from pathlib import Path
from datetime import datetime
from tempfile import gettempdir
import numpy as np
import rclpy, rosbag2_py
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray

D = Path(__file__).resolve().parent
from supervisor import Supervisor

S = json.loads((D / "settings.json").read_text())


def main(solver, task):
    cfg = json.loads((D / ("recipe_" + task + ".json")).read_text())
    period = 0.002
    os.environ.setdefault("ROS_DOMAIN_ID", "87")
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    path = Path(os.environ.get("ARM_ROSBAG_TMP", gettempdir())) / (
        "full_" + task + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    )
    path.parent.mkdir(exist_ok=True, parents=True)
    opt = rosbag2_py.StorageOptions(uri=path.as_posix(), storage_id="sqlite3")
    conv = rosbag2_py.ConverterOptions("cdr", "cdr")
    bag = rosbag2_py.SequentialWriter()
    bag.open(opt, conv)
    rclpy.init()
    planner = Node("pick_place_planner")
    ctrl = Node("arm_hand_controller")
    plant = Node("seven_axis_plant")
    traj = "/arm/joint_trajectory"
    feedback = "/arm_hand/feedback"
    cmd = "/arm_hand/command"
    for name, typ in [
        (traj, "trajectory_msgs/msg/JointTrajectory"),
        (feedback, "std_msgs/msg/Float64MultiArray"),
        (cmd, "std_msgs/msg/Float64MultiArray"),
    ]:
        bag.create_topic(
            rosbag2_py.TopicMetadata(
                id=0, name=name, type=typ, serialization_format="cdr"
            )
        )
    p = planner.create_publisher(JointTrajectory, traj, 1)
    f = plant.create_publisher(Float64MultiArray, feedback, 5)
    u = ctrl.create_publisher(Float64MultiArray, cmd, 5)
    rec = None
    count = 0
    supervis = None

    def loadpath(msg):
        nonlocal rec, supervis
        assert list(msg.joint_names) == S["names"]
        rec = np.array([list(pt.positions) + list(pt.velocities) for pt in msg.points])
        timeline = json.loads((D / (task + "_timeline.json")).read_text())
        spools = np.load(D / (task + "_spools.npy"))
        supervis = Supervisor(cfg, np.column_stack((rec, spools)), timeline, period)
        bag.write(traj, serialize_message(msg), 0)

    ctrl.create_subscription(JointTrajectory, traj, loadpath, 1)

    def control(msg):
        # Feedback packet carries encoder readings, proximal tension and feed-forward spool references.
        seq = int(msg.data[0])
        t, measured, release = msg.data[1:4]
        hand = list(msg.data[4:10])
        ref, halt = supervis.step(t, measured, list(msg.data[10:17]))
        v = Float64MultiArray(data=[float(seq), *ref.tolist(), float(halt)])
        u.publish(v)
        bag.write(feedback, serialize_message(msg), 1 + round(t * 1e9))

    ctrl.create_subscription(Float64MultiArray, feedback, control, 5)

    def send(msg):
        nonlocal count
        proc.stdin.write(json.dumps(list(msg.data)) + "\n")
        proc.stdin.flush()
        count += 1
        bag.write(cmd, serialize_message(msg), 2 + round(msg.data[0] * period * 1e9))

    plant.create_subscription(Float64MultiArray, cmd, send, 5)
    executor = rclpy.executors.SingleThreadedExecutor()
    for node in [planner, ctrl, plant]:
        executor.add_node(node)
    env = os.environ.copy()
    for key in ["PYTHONHOME", "PYTHONPATH", "CONDA_PREFIX", "VIRTUAL_ENV"]:
        env.pop(key, None)
    env["PATH"] = os.pathsep.join(
        x for x in env["PATH"].split(os.pathsep) if ".pixi" not in x.lower()
    )
    # The external target planner uses the same verified controller limits as the simulator.
    subprocess.run(
        [solver, str(D / "ros_worker.py"), task, "plan"], cwd=D, env=env, check=True
    )
    data = np.load(D / (task + "_target.npy"))
    msg = JointTrajectory()
    msg.joint_names = S["names"]
    for k, row in enumerate(data):
        pt = JointTrajectoryPoint(
            positions=row[:7].tolist(),
            velocities=row[7:14].tolist(),
            time_from_start=Duration(sec=k // 500, nanosec=(k % 500) * 2000000),
        )
        msg.points.append(pt)
    deadline = time.monotonic() + 25
    while (
        min(
            p.get_subscription_count(),
            f.get_subscription_count(),
            u.get_subscription_count(),
        )
        == 0
    ):
        executor.spin_once(timeout_sec=0.005)
        if time.monotonic() > deadline:
            raise TimeoutError("discovery")
    p.publish(msg)
    while rec is None:
        executor.spin_once(timeout_sec=0.005)
        if time.monotonic() > deadline:
            raise TimeoutError("trajectory transfer")
    proc = subprocess.Popen(
        [solver, "-u", str(D / "ros_worker.py"), task, "solve"],
        cwd=D,
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )
    q = queue.Queue()

    def read():
        for line in proc.stdout:
            if line.startswith("@"):
                q.put(json.loads(line[1:]))
        q.put(None)

    threading.Thread(target=read, daemon=True).start()
    last = time.monotonic()
    try:
        while True:
            try:
                item = q.get_nowait()
            except queue.Empty:
                item = False
            if item is None:
                break
            if item is not False:
                f.publish(Float64MultiArray(data=list(map(float, item))))
                last = time.monotonic()
            executor.spin_once(timeout_sec=0.001)
            if time.monotonic() - last > 40:
                raise TimeoutError("plant stalled")
        if proc.wait(timeout=5):
            raise RuntimeError("plant failed")
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait()
        del bag
        for node in [planner, ctrl, plant]:
            node.destroy_node()
        rclpy.shutdown()
    reader = rosbag2_py.SequentialReader()
    reader.open(opt, conv)
    ns = {}
    end = {}
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        v = deserialize_message(
            raw, JointTrajectory if topic == traj else Float64MultiArray
        )
        ns[topic] = ns.get(topic, 0) + 1
        if topic != traj:
            seq = int(v.data[0])
            assert seq == end.get(topic, -1) + 1
            end[topic] = seq
        else:
            assert len(v.points) == len(data)
    del reader
    assert ns[traj] == 1 and ns[cmd] == count == ns[feedback] == len(data)
    shutil.copytree(path, D / path.name)
    (D / (task + "_ros_verify.json")).write_text(
        json.dumps(
            {"distribution": os.getenv("ROS_DISTRO"), "bag": path.name, "messages": ns},
            indent=2,
        )
    )


if __name__ == "__main__":
    main(*sys.argv[1:])
