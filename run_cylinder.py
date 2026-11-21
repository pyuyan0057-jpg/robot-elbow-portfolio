"""Recompute the C2 and C4 cylinder tasks in an isolated Python environment."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import venv

ROOT = Path(__file__).resolve().parent
STUDY = ROOT / "studies" / "seven-axis-control-c4"


def main():
    if sys.version_info[:2] != (3, 11) or sys.platform not in ("win32", "linux"):
        raise SystemExit("Use 64-bit Python 3.11 on Windows or Linux.")
    compiler = os.environ.get("CXX", "g++")
    if not shutil.which(compiler):
        raise SystemExit("Install g++ (C++17) or set CXX to its executable path.")
    environment = ROOT / ".venv" / "cylinder"
    python = environment / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    if not python.is_file():
        venv.EnvBuilder(with_pip=True).create(environment)
    subprocess.run([str(python), "-m", "pip", "install", "--disable-pip-version-check",
                    "-r", str(STUDY / "requirements.txt")], check=True)
    subprocess.run([str(python), str(STUDY / "compare_cylinder.py")], check=True)


if __name__ == "__main__":
    main()
