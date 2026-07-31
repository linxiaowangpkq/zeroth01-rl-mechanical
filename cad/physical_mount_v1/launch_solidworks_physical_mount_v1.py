from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).with_name(
    "create_solidworks_physical_mount_v1.py"
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Launch the bounded SolidWorks physical-mount build without "
            "inheriting the calling terminal handles."
        )
    )
    _, build_args = parser.parse_known_args()
    command = [sys.executable, str(SCRIPT), *build_args]
    process = subprocess.Popen(
        command,
        cwd=str(SCRIPT.parents[5]),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        close_fds=True,
        creationflags=(
            subprocess.CREATE_NEW_CONSOLE
            | subprocess.CREATE_NEW_PROCESS_GROUP
        ),
    )
    print(process.pid, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
