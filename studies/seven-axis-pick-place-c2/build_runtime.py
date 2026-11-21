"""Build the small MuJoCo stepping library with the same headers as Python."""

import os
import shutil
import subprocess
import sys
from pathlib import Path

import mujoco


def build():
    directory = Path(__file__).resolve().parent
    sdk = Path(mujoco.__file__).resolve().parent
    windows = os.name == "nt"
    if sys.platform != "linux" and not windows:
        raise RuntimeError("This build entry point supports Windows and Linux.")
    compiler = os.environ.get("CXX") or shutil.which("g++")
    if not compiler:
        raise RuntimeError("Install a C++17 g++ compiler or set CXX to its path.")
    name = "dynamics.dll" if windows else "dynamics.so"
    target = directory / name
    library = sdk / "mujoco.dll" if windows else next(sdk.glob("libmujoco.so*"))
    flags = (
        ["-static-libgcc", "-static-libstdc++"]
        if windows
        else ["-fPIC", "-Wl,-rpath," + str(sdk)]
    )
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-O2",
            "-shared",
            *flags,
            "-I" + str(sdk / "include"),
            str(directory / "dynamics.cpp"),
            str(library),
            "-o",
            str(target),
        ],
        check=True,
    )
    return target


if __name__ == "__main__":
    print(build())
