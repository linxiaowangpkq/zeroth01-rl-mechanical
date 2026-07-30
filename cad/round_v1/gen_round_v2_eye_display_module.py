from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from round_v1_common import eye_display_module


def gen_step():
    return eye_display_module()
