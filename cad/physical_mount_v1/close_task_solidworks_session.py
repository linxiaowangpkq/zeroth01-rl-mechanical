from __future__ import annotations

import importlib.util
from pathlib import Path

import pythoncom


ROOT = Path(__file__).resolve().parents[2]
GENERATOR = Path(__file__).with_name("create_solidworks_physical_mount_v1.py")


def load_generator():
    spec = importlib.util.spec_from_file_location(
        "physical_mount_solidworks_generator",
        GENERATOR,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {GENERATOR}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    pythoncom.CoInitialize()
    try:
        generator = load_generator()
        sessions = list(generator.iter_sw_apps())
        if len(sessions) != 1:
            raise RuntimeError(
                f"expected exactly one SolidWorks session, got {len(sessions)}"
            )
        _, sw = sessions[0]
        documents = []
        allowed_root = str(
            (ROOT / "generated" / "solidworks").resolve()
        ).lower()
        for raw in list(generator.base.call(sw, "GetDocuments", []) or []):
            model = generator.base.as_model_doc(raw)
            title = str(generator.base.call(model, "GetTitle", ""))
            path = str(generator.base.call(model, "GetPathName", ""))
            allowed = (
                (path and path.lower().startswith(allowed_root))
                or (not path and title in {"装配体1", "Assembly1"})
            )
            documents.append((title, path, allowed))
        refused = [
            {"title": title, "path": path}
            for title, path, allowed in documents
            if not allowed
        ]
        if refused:
            raise RuntimeError(
                f"refusing to close non-task documents: {refused}"
            )
        for title, _, _ in documents:
            sw.CloseDoc(title)
        sw.ExitApp()
        print(
            "closed task SolidWorks session: "
            + ", ".join(title for title, _, _ in documents)
        )
        return 0
    finally:
        pythoncom.CoFreeUnusedLibraries()
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    raise SystemExit(main())
