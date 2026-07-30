from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from round_v1_common import arm_sleeve


def gen_step():
    return arm_sleeve("left", "upper")
