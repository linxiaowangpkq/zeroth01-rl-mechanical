from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from build123d import Compound

from round_v1_common import round_v1_assembly


def gen_step():
    assembly = round_v1_assembly()
    return Compound(
        label="ZEROTH01_ROUND_V3_WHITE_EVA_16_BLUE_SERVOS_ASSEMBLY",
        children=list(assembly.children),
    )
