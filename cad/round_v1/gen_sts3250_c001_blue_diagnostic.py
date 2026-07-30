from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from round_v1_common import sts3250_controlled_case


def gen_step():
    return sts3250_controlled_case()
