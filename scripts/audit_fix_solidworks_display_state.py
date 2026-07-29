from __future__ import annotations

import json
from pathlib import Path

import pythoncom
import win32com.client as win32


ROOT = Path(__file__).resolve().parents[1]
ASSEMBLY = (
    ROOT
    / "generated"
    / "solidworks"
    / "OPEN_FIRST_ZEROTH01_16DOF_KINEMATIC_REVIEW.SLDASM"
)
REPORT = ROOT / "reports" / "solidworks_display_state_audit.json"
SNAPSHOT_ISOMETRIC = (
    ROOT
    / "snapshots"
    / "solidworks"
    / "zeroth01_neutral_display_state_refreshed.png"
)
SNAPSHOT_ROBOT_FRONT = (
    ROOT
    / "snapshots"
    / "solidworks"
    / "zeroth01_neutral_robot_front_refreshed.png"
)
SNAPSHOT_ROBOT_SIDE = (
    ROOT
    / "snapshots"
    / "solidworks"
    / "zeroth01_neutral_robot_side_refreshed.png"
)

SW_DOC_ASSEMBLY = 2
SW_SAVE_AS_CURRENT_VERSION = 0
SW_SAVE_AS_SILENT = 1
IID_IMODELDOC2 = "{B90793FB-EF3D-4B80-A5C4-99959CDB6CEB}"
IID_IASSEMBLYDOC = "{83A33D35-27C5-11CE-BFD4-00400513BB57}"


def value_or_call(obj, name: str, *args):
    value = getattr(obj, name)
    return value(*args) if callable(value) else value


def transform_array(transform) -> list[float] | None:
    if transform is None:
        return None
    try:
        return [float(value) for value in list(transform.ArrayData)]
    except Exception:
        return None


def max_transform_delta(
    first: list[float] | None, second: list[float] | None
) -> float | None:
    if first is None or second is None or len(first) != len(second):
        return None
    return max(abs(first[index] - second[index]) for index in range(len(first)))


def get_exploded_view_names(assembly, configuration: str) -> list[str]:
    for getter in (
        lambda: assembly.GetExplodedViewNames2(configuration),
        lambda: assembly.GetExplodedViewNames(),
    ):
        try:
            raw = getter()
            if raw is None:
                return []
            if isinstance(raw, str):
                return [raw]
            return [str(value) for value in list(raw)]
        except Exception:
            continue
    return []


def get_is_exploded(model) -> bool | None:
    # The obsolete no-output-argument call is more reliable through late-bound
    # pywin32 than IModelDocExtension.IsExploded(out view_name).
    try:
        return bool(model.IsExploded())
    except Exception:
        return None


def save_view(model, view_id: int, path: Path) -> bool:
    model.ShowNamedView2("", view_id)
    model.ViewZoomtofit2()
    model.GraphicsRedraw2()
    model.SaveAs3(str(path), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT)
    return path.is_file() and path.stat().st_size > 0


def main() -> None:
    pythoncom.CoInitialize()
    sw = win32.GetActiveObject("SldWorks.Application")
    raw_model = sw.ActiveDoc
    if raw_model is None or int(value_or_call(raw_model, "GetType")) != SW_DOC_ASSEMBLY:
        raise RuntimeError("the active SolidWorks document is not an assembly")
    model_module = win32.gencache.GetModuleForCLSID(IID_IMODELDOC2)
    assembly_module = win32.gencache.GetModuleForCLSID(IID_IASSEMBLYDOC)
    model = model_module.IModelDoc2(raw_model._oleobj_)
    assembly = assembly_module.IAssemblyDoc(raw_model._oleobj_)
    active_path = Path(str(value_or_call(model, "GetPathName"))).resolve()
    if active_path != ASSEMBLY.resolve():
        raise RuntimeError(
            f"unexpected active assembly: {active_path}; expected {ASSEMBLY}"
        )

    components = list(value_or_call(assembly, "GetComponents", False) or [])
    configuration_manager = model.ConfigurationManager
    configuration = str(configuration_manager.ActiveConfiguration.Name)
    exploded_names = get_exploded_view_names(assembly, configuration)
    is_exploded_before = get_is_exploded(model)
    try:
        presentation_enabled_before = bool(assembly.EnablePresentation)
    except Exception:
        presentation_enabled_before = None

    component_rows: list[dict[str, object]] = []
    presentation_count = 0
    exploded_transform_count = 0
    for component in components:
        name = str(component.Name2)
        transform = transform_array(component.Transform2)
        try:
            presentation = transform_array(component.PresentationTransform)
        except Exception:
            presentation = None
        collapsed = transform_array(component.GetSpecificTransform(True))
        exploded = transform_array(component.GetSpecificTransform(False))
        presentation_count += presentation is not None
        exploded_delta = max_transform_delta(collapsed, exploded)
        exploded_transform_count += (
            exploded_delta is not None and exploded_delta > 1e-10
        )
        component_rows.append(
            {
                "component": name,
                "presentation_transform_present_before": presentation is not None,
                "transform_to_collapsed_max_abs_delta": max_transform_delta(
                    transform, collapsed
                ),
                "collapsed_to_exploded_max_abs_delta": exploded_delta,
            }
        )

    # SOLIDWORKS documents that presentation transforms affect only graphics.
    # Remove them before disabling presentation mode so the next draw follows
    # each component's underlying Transform2.
    try:
        assembly.EnablePresentation = True
    except Exception:
        pass
    for component in components:
        try:
            component.RemovePresentationTransform()
        except Exception:
            pass
    try:
        assembly.EnablePresentation = False
    except Exception:
        pass

    collapse_results: dict[str, bool] = {}
    for name in exploded_names:
        try:
            collapse_results[name] = bool(assembly.ShowExploded2(False, name))
        except Exception:
            collapse_results[name] = False
    try:
        assembly.ViewCollapseAssembly()
    except Exception:
        pass

    try:
        assembly.UpdateBox()
    except Exception:
        pass
    try:
        model.EditRebuild3()
    except Exception:
        pass
    try:
        model.ForceRebuild3(False)
    except Exception:
        pass
    model.GraphicsRedraw2()
    snapshot_results = {
        "isometric": save_view(model, 7, SNAPSHOT_ISOMETRIC),
        # Zeroth-01 uses URDF Z-up and Y-forward. The SolidWorks Bottom view
        # projects the X-Z plane with +Z upward on screen.
        "robot_front": save_view(model, 6, SNAPSHOT_ROBOT_FRONT),
        # The SolidWorks Right view projects the Y-Z plane.
        "robot_side": save_view(model, 4, SNAPSHOT_ROBOT_SIDE),
    }
    # Leave the saved and active assembly in the useful upright robot front
    # view, rather than whichever auxiliary snapshot happened to run last.
    model.ShowNamedView2("", 6)
    model.ViewZoomtofit2()
    model.GraphicsRedraw2()
    model.SaveAs3(
        str(ASSEMBLY), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT
    )

    is_exploded_after = get_is_exploded(model)
    try:
        presentation_enabled_after = bool(assembly.EnablePresentation)
    except Exception:
        presentation_enabled_after = None

    payload = {
        "assembly": str(active_path),
        "configuration": configuration,
        "component_count": len(components),
        "presentation_enabled_before": presentation_enabled_before,
        "presentation_transform_count_before": presentation_count,
        "is_exploded_before": is_exploded_before,
        "exploded_view_names": exploded_names,
        "components_with_collapsed_vs_exploded_delta": exploded_transform_count,
        "collapse_results": collapse_results,
        "presentation_enabled_after": presentation_enabled_after,
        "is_exploded_after": is_exploded_after,
        "snapshots": {
            "isometric": str(SNAPSHOT_ISOMETRIC),
            "robot_front": str(SNAPSHOT_ROBOT_FRONT),
            "robot_side": str(SNAPSHOT_ROBOT_SIDE),
        },
        "snapshot_results": snapshot_results,
        "components": component_rows,
    }
    REPORT.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
