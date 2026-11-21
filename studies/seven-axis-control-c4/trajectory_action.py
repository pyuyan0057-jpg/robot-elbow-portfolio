"""FollowJointTrajectory task adapter for a synchronized, preplanned seven-axis grasp."""
import numpy as np
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from control_msgs.action import FollowJointTrajectory as FJT
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTolerance

class TaskAction:
    def __init__(self, node, planner, executor, names, tape, period, loadpath, mode):
        self.names=list(names)
        self.tape=tape
        self.dt=period
        self.loadpath=loadpath
        self.mode=mode
        self.handle=None
        self.client_goal=None
        self.last_actual=None
        self.done=Future(executor=executor)
        self.first_fault=None
        self.cancel_request=None
        self.feedback_count=0
        self.last_feedback_time=-1
        self.entered=False
        topic='/arm/follow_joint_trajectory'
        self.server=ActionServer(node,FJT,topic,execute_callback=self.execute,
                    goal_callback=self.validate,cancel_callback=lambda _:CancelResponse.ACCEPT,
                    callback_group=ReentrantCallbackGroup())
        self.client=ActionClient(planner,FJT,topic)
    def validate(self,goal):
        traj=goal.trajectory
        if self.entered or list(traj.joint_names)!=self.names or len(traj.points)!=len(self.tape):
            return GoalResponse.REJECT
        if (traj.header.stamp.sec or traj.header.stamp.nanosec or goal.multi_dof_trajectory.points
            or goal.component_path_tolerance or goal.component_goal_tolerance
            or goal.goal_time_tolerance.sec or goal.goal_time_tolerance.nanosec):
            return GoalResponse.REJECT
        for tolerances in [goal.path_tolerance,goal.goal_tolerance]:
            seen=set()
            for tol in tolerances:
                if (tol.name not in self.names or tol.name in seen or not np.isfinite(tol.position)
                    or (tol.position<0 and tol.position!=-1) or tol.velocity or tol.acceleration):
                    return GoalResponse.REJECT
                seen.add(tol.name)
        for i,pt in enumerate(traj.points):
            stamp=pt.time_from_start.sec*10**9+pt.time_from_start.nanosec
            if (stamp!=round(i*self.dt*1e9) or pt.accelerations or pt.effort
                or list(pt.positions)!=self.tape[i,:7].tolist() or list(pt.velocities)!=self.tape[i,7:].tolist()):
                return GoalResponse.REJECT
        self.entered=True
        return GoalResponse.ACCEPT
    def bounds(self,items,default):
        out=np.full(7,default)
        for tol in items:
            out[self.names.index(tol.name)]=np.inf if tol.position==-1 else (tol.position or default)
        return out
    async def execute(self,handle):
        self.handle=handle
        self.path=self.bounds(handle.request.path_tolerance,.035)
        self.goal=self.bounds(handle.request.goal_tolerance,.018)
        self.loadpath(handle.request.trajectory)
        return await self.done
    def send(self,msg):
        goal=FJT.Goal()
        goal.trajectory=msg
        if self.mode=='path_fault':
            goal.path_tolerance=[JointTolerance(name=self.names[1],position=1e-7)]
        if self.mode=='goal_fault':
            goal.goal_tolerance=[JointTolerance(name=self.names[1],position=1e-9)]
        return self.client.send_goal_async(goal,feedback_callback=self.feedback)
    def feedback(self,message):
        point=message.feedback
        assert list(point.joint_names)==self.names and len(point.actual.positions)==7
        self.feedback_count+=1
        self.last_feedback_time=point.actual.time_from_start.sec+point.actual.time_from_start.nanosec*1e-9
    def step(self,seq,t,q,supervisor):
        actual=np.asarray(q)
        error=self.tape[seq,:7]-actual
        self.last_actual=actual
        trigger=.12 if self.mode=='cancel' else supervisor.times[1]+.30
        if self.mode in ('cancel','cancel_loaded') and t>=trigger and self.cancel_request is None:
            self.cancel_request=self.client_goal.cancel_goal_async()
        if self.first_fault is None:
            if self.handle.is_cancel_requested:
                self.first_fault=('cancel',t)
            elif np.any(abs(error)>self.path):
                self.first_fault=('path',t)
            if self.first_fault:
                supervisor.stop(t,actual)
        if seq%25==0 or seq==len(self.tape)-1:
            msg=FJT.Feedback()
            msg.joint_names=self.names
            for field,values in [('desired',self.tape[seq,:7]),('actual',actual),('error',error)]:
                setattr(msg,field,JointTrajectoryPoint(positions=values.tolist(),time_from_start=Duration(sec=seq//500,nanosec=seq%500*2000000)))
            self.handle.publish_feedback(msg)
    def finish(self,supervisor):
        result=FJT.Result()
        issue=self.first_fault
        if issue and issue[0]=='cancel':
            self.handle.canceled()
            result.error_string='Canceled; task stop routine completed.'
        elif issue or supervisor.fault:
            self.handle.abort()
            result.error_code=FJT.Result.PATH_TOLERANCE_VIOLATED
            result.error_string=issue[0] if issue else supervisor.fault
        elif np.any(abs(self.last_actual-self.tape[-1,:7])>self.goal):
            self.handle.abort()
            result.error_code=FJT.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string='Terminal joint tolerance exceeded.'
        else:
            self.handle.succeed()
            result.error_code=FJT.Result.SUCCESSFUL
        self.done.set_result(result)
