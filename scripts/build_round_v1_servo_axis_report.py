from __future__ import annotations

from pathlib import Path
import sys


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
CAD_DIR = ROOT / "cad" / "round_v1"
sys.path.insert(0, str(CAD_DIR))

from round_v1_common import servo_instances, write_servo_axis_report


def main() -> int:
    _, rows = servo_instances()
    if len(rows) != 16 or not all(row["gate"] == "PASS" for row in rows):
        raise RuntimeError("STS3250 joint-axis placement gate failed")
    write_servo_axis_report(rows)
    print(f"SERVO_AXIS_ROWS={len(rows)}")
    print("PARENT_HOUSING_CHILD_OUTPUT_SEMANTICS=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
