"""Print selected world-space SolidWorks component boxes for fit diagnosis."""

from __future__ import annotations

import json

import pythoncom
import win32com.client


TARGETS = (
    "BODY_SKELETON_TOP_TRIMMED",
    "CORES3_K128_PURCHASED_ENVELOPE",
    "CORES3_INTERNAL_TORSO_CRADLE",
)


def value(obj, name):
    item = getattr(obj, name)
    return item() if callable(item) else item


def main() -> int:
    pythoncom.CoInitialize()
    sw = win32com.client.Dispatch("SldWorks.Application")
    document = sw.ActiveDoc
    if document is None:
        raise RuntimeError("open the v3 SolidWorks assembly first")
    rows = []
    for component in document.GetComponents(False) or []:
        name = str(value(component, "Name2"))
        if not any(token in name for token in TARGETS):
            continue
        box = list(component.GetBox(False, False) or [])
        transform = list(value(component.Transform2, "ArrayData") or [])
        rows.append(
            {
                "component": name,
                "bbox_m": [float(item) for item in box],
                "transform_array": [float(item) for item in transform],
            }
        )
    print(json.dumps(rows, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
