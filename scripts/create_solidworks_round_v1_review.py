"""Workspace-local compatibility shim for the validated STEP importer.

The historical helper remains under the canonical Zeroth reference workspace;
v4 imports it without copying or modifying upstream-derived implementation.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path


TARGET = (
    Path(__file__).resolve().parents[3]
    / "reference"
    / "zeroth01"
    / "scripts"
    / "create_solidworks_round_v1_review.py"
)
SPEC = importlib.util.spec_from_file_location("zeroth01_validated_round_sw", TARGET)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(TARGET)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
for NAME in dir(MODULE):
    if not NAME.startswith("__"):
        globals()[NAME] = getattr(MODULE, NAME)
