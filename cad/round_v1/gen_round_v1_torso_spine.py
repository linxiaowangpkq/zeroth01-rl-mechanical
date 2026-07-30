from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from round_v1_common import torso_spine


def gen_step():
    return torso_spine()
