"""Build and solve the actual SOLIDWORKS Motion assembly for Zeroth-01 v5.

Every joint uses the same physical topology:

  STS3250 case --LOCK--> parent carrier
  parent thrust ring --LOCK--> parent carrier
  output bridge --same rigid SLDPRT--> child carrier
  hip-yaw four-sleeve spacer --same rigid SLDPRT--> child carrier
  thrust-ring bore --CONCENTRIC--> output-bridge OD
  thrust-ring front --COINCIDENT--> output-bridge front

The final two mates leave exactly one rotational DoF.  Motion motors are
created on the output-bridge cylindrical faces and SOLIDWORKS-rendered frames
are used verbatim to build the GIF.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import shutil
import sys
from pathlib import Path

import pythoncom
import win32com.client as win32
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
V5_SW_SOURCE = HERE / "create_solidworks_v5.py"
R31_ROOT = ROOT.parents[1] / "hardware" / "mechanical" / "rs_true_atom_motion_r31"
sys.path.insert(0, str(R31_ROOT / "scripts"))
import create_r31_true_atom_rs00_ankle_motion as r31  # noqa: E402

MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
V5_URDF = ROOT / "generated" / "urdf" / "physical_mount_v5_original_16dof_solidworks_motion" / "zeroth01_physical_mount_v5_original_16dof_solidworks_motion.urdf"
PORTABLE = ROOT / "generated" / "solidworks" / "physical_mount_v5_original_16dof_solidworks_motion" / "portable_flat"
MOTION_LINK_STEP_DIR = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "motion_link_parts"
SOURCE_ASM = PORTABLE / "OPTIONAL_XRAY_ZEROTH01_V5_16_BLUE_STS3250_TRANSMISSION.SLDASM"
MOTION_ASM = PORTABLE / "OPEN_FIRST_ZEROTH01_V5_16DOF_ACTUAL_SOLIDWORKS_MOTION.SLDASM"
REPORT_ROOT = ROOT / "reports" / "v5_original_16dof_solidworks_motion"
MATE_CSV = REPORT_ROOT / "solidworks_motion_mates.csv"
CURVE_CSV = REPORT_ROOT / "solidworks_motion_curve.csv"
INTERFERENCE_CSV = REPORT_ROOT / "solidworks_motion_interferences.csv"
GATE_JSON = REPORT_ROOT / "solidworks_motion_gate.json"
MOTION_LINK_NATIVE_GATE = REPORT_ROOT / "solidworks_motion_link_native_gate.json"
SNAP_ROOT = ROOT / "snapshots" / "motion" / "physical_mount_v5_original_16dof_solidworks_motion"
GIF_PATH = SNAP_ROOT / "zeroth01_v5_actual_solidworks_motion.gif"
MOTION_ADDIN = Path(r"E:\SolidWorks\SOLIDWORKS\cmotionsw.dll")

SW_DOC_ASSEMBLY = 2
SW_OPEN_SILENT = 1
SW_SAVE_CURRENT = 0
SW_SAVE_SILENT = 1
SW_MATE_COINCIDENT = 0
SIM_DURATION_S = 0.40
# Despite the public RPM documentation, this installed COM/Motion build
# solves 6.0 as 6 rad/s (2.4 rad in 0.40 s).  Use the empirically verified
# native value 0.5 for a conservative 0.20-rad validation sweep.
SIM_SPEED_NATIVE = 0.5
FRAME_COUNT = 12
ISOLATED_FRAME_COUNT = 5
SW_MOTION_STUDY_TYPE_MOTION_ANALYSIS = 4


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v5sw = load(V5_SW_SOURCE, "zeroth_v5_static_solidworks_for_motion")


def manifest_data():
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def load_motion_addin(sw) -> int:
    if not MOTION_ADDIN.is_file():
        raise FileNotFoundError(MOTION_ADDIN)
    status = int(sw.LoadAddIn(str(MOTION_ADDIN)))
    print(f"SOLIDWORKS Motion add-in status={status} path={MOTION_ADDIN}", flush=True)
    return status


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    if not fields:
        fields = ["empty"]
        rows = [{"empty": ""}]
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def base_name(value: str) -> str:
    leaf = str(value or "").split("/")[-1]
    if "-" in leaf and leaf.rsplit("-", 1)[1].isdigit():
        leaf = leaf.rsplit("-", 1)[0]
    return leaf


def open_motion_copy(sw):
    v5sw.configure()
    v5sw.v3.close_task_documents(sw)
    raw = v5sw.v3.v1.first(sw.OpenDoc6(str(SOURCE_ASM), SW_DOC_ASSEMBLY, SW_OPEN_SILENT, "", 0, 0))
    if raw is None:
        raise RuntimeError(SOURCE_ASM)
    model = r31.swp.as_model_doc(raw)
    code = int(model.SaveAs3(str(MOTION_ASM), SW_SAVE_CURRENT, SW_SAVE_SILENT))
    if code < 0 or not MOTION_ASM.is_file():
        raise RuntimeError(f"SaveAs3 motion copy failed: {code}")
    asm = r31.swp.as_assembly_doc(raw)
    asm.ResolveAllLightWeightComponents(True)
    model.ForceRebuild3(False)
    return model, asm


def reopen_motion(sw):
    r31.FACE_CACHE.clear()
    v5sw.v3.close_task_documents(sw)
    raw = v5sw.v3.v1.first(sw.OpenDoc6(str(MOTION_ASM), SW_DOC_ASSEMBLY, SW_OPEN_SILENT, "", 0, 0))
    if raw is None:
        raise RuntimeError(MOTION_ASM)
    model = r31.swp.as_model_doc(raw)
    asm = r31.swp.as_assembly_doc(raw)
    asm.ResolveAllLightWeightComponents(True)
    model.ForceRebuild3(False)
    return model, asm


def components_by_manifest_id(asm):
    # SOLIDWORKS may replace an assigned occurrence Name2 with its filename
    # stem after SaveAs/reopen.  Match by portable native filename and exact
    # installed shaft-frame translation instead; this is also independently
    # checked by the static component-transform gate.
    available = []
    for raw in list(asm.GetComponents(False) or []):
        comp = r31.dispatch(raw)
        available.append(
            {
                "component": comp,
                "filename": Path(r31.component_path(comp)).name.lower(),
                "translation_mm": r31.arr_translation_mm(component_transform_array(comp)),
            }
        )
    result = {}
    for row in manifest_data()["components"]:
        component_id = str(row["component_id"])
        filename = v5sw.native_part_for(row).name.lower()
        expected = tuple(float(row["transform_local_mm_to_world_mm"][index][3]) for index in range(3))
        candidates = [item for item in available if item["filename"] == filename]
        if not candidates:
            raise RuntimeError(f"no occurrence with native file {filename} for {component_id}")
        best = min(candidates, key=lambda item: r31.vec_len(r31.vec_sub(item["translation_mm"], expected)))
        error = r31.vec_len(r31.vec_sub(best["translation_mm"], expected))
        if error > 0.05:
            raise RuntimeError(f"occurrence transform mismatch {component_id}: {error:.6f} mm")
        available.remove(best)
        result[component_id] = best["component"]
    if available:
        raise RuntimeError(f"unmapped SOLIDWORKS occurrences: {len(available)}")
    return result


def owner_carriers(data):
    result = {}
    for row in data["components"]:
        if row["role"] == "source_load_bearing_carrier":
            result[str(row["owner_link"])] = str(row["component_id"])
    return result


def safe(value: str) -> str:
    return "".join(character if character.isalnum() else "_" for character in value)


def motion_link_step(owner: str) -> Path:
    return MOTION_LINK_STEP_DIR / f"ZEROTH01_V5_MOTION_LINK_{safe(owner).upper()}.step"


def motion_link_native(owner: str) -> Path:
    return PORTABLE / f"ZEROTH01_V5_MOTION_LINK_{safe(owner).upper()}.SLDPRT"


def carrier_order(data) -> list[str]:
    return [
        str(row["owner_link"])
        for row in data["components"]
        if row["role"] == "source_load_bearing_carrier"
    ]


def import_motion_link_parts(sw, force: bool = False) -> dict[str, object]:
    v5sw.configure()
    data = manifest_data()
    if not force and MOTION_LINK_NATIVE_GATE.is_file():
        cached = json.loads(MOTION_LINK_NATIVE_GATE.read_text(encoding="utf-8"))
        current = all(
            motion_link_native(owner).is_file()
            and motion_link_native(owner).stat().st_mtime >= motion_link_step(owner).stat().st_mtime
            for owner in carrier_order(data)
        )
        if cached.get("overall") == "PASS" and current:
            print("reuse 17/17 cached SOLIDWORKS native motion-link solid gates", flush=True)
            return cached
    rows = []
    previous = int(
        sw.GetUserPreferenceIntegerValue(v5sw.v3.round_sw.SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING)
    )
    sw.SetUserPreferenceIntegerValue(
        v5sw.v3.round_sw.SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING,
        v5sw.v3.round_sw.SW_IMPORT_NEUTRAL_AS_MULTIBODY_PART,
    )
    try:
        for index, owner in enumerate(carrier_order(data), start=1):
            source = motion_link_step(owner)
            target = motion_link_native(owner)
            if not source.is_file():
                raise FileNotFoundError(source)
            # The shared STEP importer only checks its explicit ``force`` flag.
            # Propagate our per-file freshness decision so a newly generated
            # motion-link STEP can never silently reuse an older SLDPRT.
            part_force = (
                force
                or not target.is_file()
                or target.stat().st_mtime < source.stat().st_mtime
            )
            mode = "forced-refresh" if part_force else "reuse-current"
            print(f"SOLIDWORKS motion-link import {index}/17 {owner} [{mode}]", flush=True)
            result = v5sw.v3.round_sw.import_step_part(sw, source, target, force=part_force)
            raw = v5sw.v3.v1.first(sw.OpenDoc6(str(target), 1, SW_OPEN_SILENT, "", 0, 0))
            if raw is None:
                raise RuntimeError(f"cannot open motion link native part: {target}")
            part = v5sw.v3.v1.base.as_part_doc(raw)
            solids = len(list(part.GetBodies2(0, True) or []))
            surfaces = len(list(part.GetBodies2(1, True) or []))
            v5sw.v3.v1.base.close_document(sw, raw)
            row = {
                "owner_link": owner,
                "source_step": source.relative_to(ROOT).as_posix(),
                "native_part": target.relative_to(ROOT).as_posix(),
                "solid_body_count": solids,
                "surface_body_count": surfaces,
                "status": "PASS" if solids >= 1 else "FAIL",
                **result,
            }
            rows.append(row)
            if row["status"] != "PASS":
                raise RuntimeError(row)
    finally:
        sw.SetUserPreferenceIntegerValue(
            v5sw.v3.round_sw.SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING,
            previous,
        )
    payload = {
        "schema": "zeroth01.v5.solidworks_motion_link_native_gate.v1",
        "link_count": len(rows),
        "solid_body_count": sum(int(row["solid_body_count"]) for row in rows),
        "moving_links_all_native_solids": all(row["status"] == "PASS" for row in rows),
        "links": rows,
        "overall": "PASS" if len(rows) == 17 and all(row["status"] == "PASS" for row in rows) else "FAIL",
    }
    MOTION_LINK_NATIVE_GATE.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return payload


def link_components_by_owner(asm):
    data = manifest_data()
    available = [r31.dispatch(raw) for raw in list(asm.GetComponents(True) or [])]
    result = {}
    for owner in carrier_order(data):
        filename = motion_link_native(owner).name.lower()
        candidates = [comp for comp in available if Path(r31.component_path(comp)).name.lower() == filename]
        if len(candidates) != 1:
            raise RuntimeError(f"top-level motion link occurrence mismatch {owner}: {len(candidates)}")
        result[owner] = candidates[0]
        available.remove(candidates[0])
    if available:
        raise RuntimeError(f"unexpected top-level motion components: {len(available)}")
    return result


def component_transform_array(comp):
    return [float(value) for value in list(comp.Transform2.ArrayData)]


def local_z_axis_and_origin(row):
    matrix = row["transform_local_mm_to_world_mm"]
    axis = tuple(float(matrix[index][2]) for index in range(3))
    origin = tuple(float(matrix[index][3]) for index in range(3))
    return r31.vec_unit(axis), origin


def component_solid_bodies(component):
    """Return actual assembly-context solid bodies for a multibody part.

    IComponent2.GetBody is unreliable through late-bound pywin32 for a
    multibody part: in this SOLIDWORKS build it can return the COM method
    dispatch itself.  GetBodies2 is the documented component-level API and
    returns the assembly-context bodies needed for selecting mate faces.
    """

    bodies = []
    for getter in ("GetBodies2", "GetBodies"):
        try:
            found = getattr(component, getter)(r31.SW_SOLID_BODY)
            if found:
                bodies.extend(list(found))
        except Exception:
            pass
    seen = set()
    unique = []
    for body in bodies:
        try:
            key = int(body.GetID())
        except Exception:
            key = id(body)
        if key in seen:
            continue
        seen.add(key)
        unique.append(body)
    return unique


def body_is_near_world_point(component, body, target_mm, margin_mm=30.0):
    if target_mm is None:
        return True
    try:
        box = [float(value) for value in list(r31.maybe_call(body, "GetBodyBox"))]
        if len(box) != 6:
            return True
        arr = component_transform_array(component)
        xs = (box[0], box[3])
        ys = (box[1], box[4])
        zs = (box[2], box[5])
        corners = [
            r31.world_point(arr, (x, y, z))
            for x in xs for y in ys for z in zs
        ]
        for index in range(3):
            low = min(point[index] for point in corners) - margin_mm
            high = max(point[index] for point in corners) + margin_mm
            if not low <= target_mm[index] <= high:
                return False
        return True
    except Exception:
        # Selection correctness wins over optimization if a body box is not
        # available for an imported body.
        return True


def body_local_bbox_mm(body):
    try:
        values = [float(value) * 1000.0 for value in list(r31.maybe_call(body, "GetBodyBox"))]
        return values if len(values) == 6 else None
    except Exception:
        return None


def interface_candidate_bodies(component, target_mm, radius_mm):
    """Identify the deliberately modelled ring/bridge body by its envelope."""

    # v5 CAD source facts: parent ring = OD25 x 2.00; child bridge =
    # OD19.95 x 2.05.  Sorting makes the test independent of X/Y/Z joint axis.
    expected = (2.0, 25.0, 25.0) if radius_mm > 10.0 else (2.05, 19.95, 19.95)
    tolerances = (0.35, 0.40, 0.40)
    matched = []
    nearby = []
    for body in component_solid_bodies(component):
        if not body_is_near_world_point(component, body, target_mm, 4.0):
            continue
        nearby.append(body)
        box = body_local_bbox_mm(body)
        if box is None:
            continue
        extents = sorted((box[3] - box[0], box[4] - box[1], box[5] - box[2]))
        if all(abs(extents[index] - expected[index]) <= tolerances[index] for index in range(3)):
            matched.append(body)
    print(
        f"interface_body_candidates r={radius_mm}: matched={len(matched)} nearby={len(nearby)}",
        flush=True,
    )
    return matched or nearby


def all_cylinder_faces(component, target_mm=None, body_margin_mm=30.0):
    faces = []
    for body in component_solid_bodies(component):
        if not body_is_near_world_point(component, body, target_mm, body_margin_mm):
            continue
        try:
            raw_faces = r31.maybe_call(body, "GetFaces") or []
        except Exception:
            continue
        for raw_face in raw_faces:
            try:
                face = r31.swp.FACE_MOD.IFace2(raw_face._oleobj_)
                surf = r31.swp.SURFACE_MOD.ISurface(face.GetSurface()._oleobj_)
                if surf.IsCylinder():
                    faces.append((face, [float(value) for value in list(surf.CylinderParams)]))
            except Exception:
                continue
    return faces


def all_plane_faces(component, target_mm=None, body_margin_mm=30.0):
    faces = []
    for body in component_solid_bodies(component):
        if not body_is_near_world_point(component, body, target_mm, body_margin_mm):
            continue
        try:
            raw_faces = r31.maybe_call(body, "GetFaces") or []
        except Exception:
            continue
        for raw_face in raw_faces:
            try:
                face = r31.swp.FACE_MOD.IFace2(raw_face._oleobj_)
                surf = r31.swp.SURFACE_MOD.ISurface(face.GetSurface()._oleobj_)
                if surf.IsPlane():
                    params = [float(value) for value in list(surf.PlaneParams)]
                    faces.append((face, params))
            except Exception:
                continue
    return faces


def all_component_faces(component):
    faces = []
    for body in component_solid_bodies(component):
        try:
            raw_faces = r31.maybe_call(body, "GetFaces") or []
        except Exception:
            continue
        for raw_face in raw_faces:
            try:
                faces.append(r31.swp.FACE_MOD.IFace2(raw_face._oleobj_))
            except Exception:
                continue
    return faces


def find_plane_face_world(component, target_mm, axis_hint, label, preferred_body=None):
    arr = component_transform_array(component)
    hint = r31.vec_unit(axis_hint)
    best = None
    best_score = 1.0e18
    if preferred_body is None:
        plane_faces = all_plane_faces(component, target_mm, 20.0)
    else:
        plane_faces = []
        try:
            raw_faces = r31.maybe_call(preferred_body, "GetFaces") or []
        except Exception:
            raw_faces = []
        for raw_face in raw_faces:
            try:
                face = r31.swp.FACE_MOD.IFace2(raw_face._oleobj_)
                surf = r31.swp.SURFACE_MOD.ISurface(face.GetSurface()._oleobj_)
                if surf.IsPlane():
                    plane_faces.append((face, [float(value) for value in list(surf.PlaneParams)]))
            except Exception:
                continue
    for face, params in plane_faces:
        normal = r31.vec_unit(r31.world_vec(arr, tuple(params[:3])))
        point = r31.world_point(arr, tuple(params[3:6]))
        alignment = abs(r31.vec_dot(normal, hint))
        distance = abs(r31.vec_dot(r31.vec_sub(target_mm, point), normal))
        if alignment < 0.92 or distance > 0.8:
            continue
        try:
            area = float(face.GetArea()) * 1.0e6
        except Exception:
            area = 0.0
        score = distance + (1.0 - alignment) * 10.0 - min(area, 1000.0) * 1.0e-6
        if score < best_score:
            best_score = score
            best = {"component": component, "face": face, "label": label, "body": preferred_body}
    if best is None:
        raise RuntimeError(f"no planar front face for {label}")
    return best


def find_cyl_face_world(
    component,
    target_point_mm,
    radius_mm,
    radius_tol=1.0,
    axis_tol=6.0,
    label="",
    axis_hint=(0.0, 1.0, 0.0),
    axis_dot_min=0.70,
):
    print(f"find_cyl_face {label} target={target_point_mm} r={radius_mm}", flush=True)
    hint = r31.vec_unit(axis_hint) if axis_hint is not None else None
    best = None
    best_score = float("inf")
    candidates = []
    interface_bodies = interface_candidate_bodies(component, target_point_mm, radius_mm)
    interface_faces = []
    for body in interface_bodies:
        try:
            raw_faces = r31.maybe_call(body, "GetFaces") or []
        except Exception:
            continue
        for raw_face in raw_faces:
            try:
                face = r31.swp.FACE_MOD.IFace2(raw_face._oleobj_)
                surf = r31.swp.SURFACE_MOD.ISurface(face.GetSurface()._oleobj_)
                if surf.IsCylinder():
                    interface_faces.append((body, face, [float(value) for value in list(surf.CylinderParams)]))
            except Exception:
                continue
    for body, face, params in interface_faces:
        origin, axis, radius = r31.cylinder_world(component, params)
        axis_dot = abs(r31.vec_dot(axis, hint)) if hint is not None else 1.0
        distance = r31.line_distance(target_point_mm, origin, axis)
        score = distance + abs(radius - radius_mm) * 3.0 + (1.0 - axis_dot) * 20.0
        candidates.append((score, distance, radius, origin, axis, axis_dot))
        if axis_dot >= axis_dot_min and distance <= axis_tol and abs(radius - radius_mm) <= radius_tol:
            if score < best_score:
                best_score = score
                best = {"component": component, "face": face, "label": label, "body": body}
    if best is None:
        candidates.sort(key=lambda item: item[0])
        summary = "; ".join(
            f"d={distance:.3f},r={radius:.3f},dot={axis_dot:.3f}"
            for _score, distance, radius, _origin, _axis, axis_dot in candidates[:8]
        )
        raise RuntimeError(f"no cylindrical face for {label}; nearest: {summary}")
    return best


def circular_edge_on_cylindrical_face(face, label):
    edges = list(r31.maybe_call(face, "GetEdges") or [])
    for edge in edges:
        try:
            curve = r31.maybe_call(edge, "GetCurve")
            if curve is not None and bool(r31.maybe_call(curve, "IsCircle")):
                return edge
        except Exception:
            continue
    # Every trimming edge of the analytic cylindrical face is circular; the
    # late-bound curve wrapper in this installation does not expose IsCircle.
    if edges:
        return edges[0]
    raise RuntimeError(f"no circular direction edge on {label}")


def find_coaxial_face_world(component, target_mm, axis_hint, label):
    hint = r31.vec_unit(axis_hint)
    best = None
    best_score = float("inf")
    for face, params in all_cylinder_faces(component, target_mm, 30.0):
        origin, axis, radius = r31.cylinder_world(component, params)
        axis_dot = abs(r31.vec_dot(axis, hint))
        distance = r31.line_distance(target_mm, origin, axis)
        if axis_dot < 0.85 or distance > 12.0 or not (0.8 <= radius <= 25.0):
            continue
        score = distance * 10.0 + (1.0 - axis_dot) * 20.0 - min(radius, 10.0) * 0.02
        if score < best_score:
            best_score = score
            best = {"face": face, "origin": origin, "axis": axis, "radius": radius, "distance": distance}
    if best is None:
        # Some upstream Zeroth load-bearing carriers are faceted imports and
        # expose no analytic cylindrical surface.  Location only identifies
        # the moved component; the rotation axis still comes from the parent
        # thrust ring, so a stable largest planar face is a valid fallback.
        largest_area = -1.0
        for face, _params in all_plane_faces(component):
            try:
                area = float(face.GetArea()) * 1.0e6
            except Exception:
                area = 0.0
            if area > largest_area:
                largest_area = area
                best = {"face": face, "origin": target_mm, "axis": hint, "radius": 0.0, "distance": 0.0}
        if best is None:
            largest_area = -1.0
            for face in all_component_faces(component):
                try:
                    area = float(face.GetArea()) * 1.0e6
                except Exception:
                    area = 0.0
                if area > largest_area:
                    largest_area = area
                    best = {"face": face, "origin": target_mm, "axis": hint, "radius": 0.0, "distance": 0.0}
        if best is None:
            raise RuntimeError(f"no child face for {label}")
    print(
        f"  child_cyl_selected label={label} d={best['distance']:.3f} r={best['radius']:.3f} "
        f"axis=({best['axis'][0]:.4f},{best['axis'][1]:.4f},{best['axis'][2]:.4f})",
        flush=True,
    )
    return best


def add_rigid_and_hinge_mates(model, asm, comps):
    data = manifest_data()
    carriers = owner_carriers(data)
    rows = []
    for key, comp in comps.items():
        r31.unfix_component(asm, model, comp, key)
    root_id = carriers["Z_BOT2_MASTER_BODY_SKELETON"]
    r31.fix_component(asm, model, comps[root_id], root_id)

    for row in data["components"]:
        role = str(row["role"])
        if role == "source_load_bearing_carrier":
            continue
        owner = str(row["owner_link"])
        if owner not in carriers:
            raise RuntimeError(f"no carrier for {owner}")
        rows.append(
            r31.add_lock_mate(
                asm,
                model,
                f"{row['component_id']}_LOCK_TO_{owner}",
                comps[str(row["component_id"])],
                comps[carriers[owner]],
            )
        )

    by_id = {str(row["component_id"]): row for row in data["components"]}
    for spec in data["joint_specs"]:
        sid = str(spec["id"])
        joint = str(spec["name"])
        ring_id = f"{sid}_PARENT_THRUST_RING_{joint}"
        bridge_id = f"{sid}_PCD14_OUTPUT_BRIDGE_{joint}"
        ring_row = by_id[ring_id]
        bridge_row = by_id[bridge_id]
        axis, origin = local_z_axis_and_origin(bridge_row)
        ring_cyl = find_cyl_face_world(comps[ring_id], origin, 10.2, 0.35, 1.0, f"{joint} parent thrust bore", axis_hint=axis, axis_dot_min=0.92)
        bridge_cyl = find_cyl_face_world(comps[bridge_id], origin, 9.975, 0.35, 1.0, f"{joint} output bridge OD", axis_hint=axis, axis_dot_min=0.92)
        rows.append(r31.add_mate_faces(asm, model, f"{joint}_CONCENTRIC_OUTPUT_AXIS", ring_cyl, bridge_cyl, r31.SW_MATE_CONCENTRIC, align=r31.SW_MATE_ALIGN_CLOSEST))
        ring_plane = find_plane_face_world(comps[ring_id], origin, axis, f"{joint} ring front")
        bridge_plane = find_plane_face_world(comps[bridge_id], origin, axis, f"{joint} bridge front")
        rows.append(r31.add_mate_faces(asm, model, f"{joint}_COINCIDENT_AXIAL_STOP", ring_plane, bridge_plane, SW_MATE_COINCIDENT, align=r31.SW_MATE_ALIGN_CLOSEST))

    model.ForceRebuild3(False)
    write_csv(MATE_CSV, rows)
    return rows


def create_motion_link_assembly(sw):
    """Create a 17-component top-level assembly, one rigid part per URDF link."""

    v5sw.configure()
    v5sw.v3.close_task_documents(sw)
    if MOTION_ASM.exists():
        MOTION_ASM.unlink()
    raw = v5sw.v3.v1.first(sw.NewDocument(str(v5sw.v3.v1.base.ASM_TEMPLATE), 0, 0, 0))
    if raw is None:
        raise RuntimeError("SOLIDWORKS NewDocument failed for motion-link assembly")
    model = r31.swp.as_model_doc(raw)
    asm = r31.swp.as_assembly_doc(raw)
    data = manifest_data()
    for index, owner in enumerate(carrier_order(data), start=1):
        path = motion_link_native(owner)
        component = asm.AddComponent5(str(path), 0, "", False, "", 0.0, 0.0, 0.0)
        if component is None:
            opened = v5sw.v3.v1.first(sw.OpenDoc6(str(path), 1, SW_OPEN_SILENT, "", 0, 0))
            v5sw.v3.v1.base.activate_document(sw, model)
            component = asm.AddComponent5(str(path), 0, "", False, "", 0.0, 0.0, 0.0)
            if opened is not None:
                v5sw.v3.v1.base.close_document(sw, opened)
                v5sw.v3.v1.base.activate_document(sw, model)
        if component is None:
            raise RuntimeError(f"cannot insert motion link: {owner}")
        transform_error = v5sw.v3.v1.base.set_component_transform(
            sw,
            component,
            (((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0)),
        )
        if transform_error > 1.0e-8:
            raise RuntimeError(f"motion link identity transform failed {owner}: {transform_error}")
        try:
            component.Name2 = f"LINK_{safe(owner)}"
        except Exception:
            pass
        if index == 1:
            fixed = r31.fix_component(asm, model, component, owner)
        else:
            fixed = r31.unfix_component(asm, model, component, owner)
        print(f"motion link occurrence {index}/17 {owner}: {fixed}; transform_error={transform_error}", flush=True)
    model.ForceRebuild3(False)
    code = int(model.SaveAs3(str(MOTION_ASM), SW_SAVE_CURRENT, SW_SAVE_SILENT))
    if code < 0 or not MOTION_ASM.is_file():
        raise RuntimeError(f"cannot save motion link assembly: {code}")
    return model, asm


def add_link_hinge_mates(model, asm, comps):
    rows = []
    data = manifest_data()
    by_id = {str(row["component_id"]): row for row in data["components"]}
    for spec in data["joint_specs"]:
        sid = str(spec["id"])
        joint = str(spec["name"])
        bridge_id = f"{sid}_PCD14_OUTPUT_BRIDGE_{joint}"
        ring_id = f"{sid}_PARENT_THRUST_RING_{joint}"
        ring_row = by_id[ring_id]
        bridge_row = by_id[bridge_id]
        ring_owner = str(ring_row["owner_link"])
        bridge_owner = str(bridge_row["owner_link"])
        axis, origin = local_z_axis_and_origin(bridge_row)
        parent_cyl = find_cyl_face_world(
            comps[ring_owner], origin, 10.2, 0.35, 1.0,
            f"{joint} parent motion-link thrust bore", axis_hint=axis, axis_dot_min=0.92,
        )
        child_cyl = find_cyl_face_world(
            comps[bridge_owner], origin, 9.975, 0.35, 1.0,
            f"{joint} child motion-link output bridge", axis_hint=axis, axis_dot_min=0.92,
        )
        rows.append(
            r31.add_mate_faces(
                asm, model, f"{joint}_CONCENTRIC_OUTPUT_AXIS", parent_cyl, child_cyl,
                r31.SW_MATE_CONCENTRIC, align=r31.SW_MATE_ALIGN_CLOSEST,
            )
        )
        parent_plane = find_plane_face_world(
            comps[ring_owner], origin, axis, f"{joint} parent link axial stop",
            preferred_body=parent_cyl["body"],
        )
        child_plane = find_plane_face_world(
            comps[bridge_owner], origin, axis, f"{joint} child link axial stop",
            preferred_body=child_cyl["body"],
        )
        rows.append(
            r31.add_mate_faces(
                asm, model, f"{joint}_COINCIDENT_AXIAL_STOP", parent_plane, child_plane,
                SW_MATE_COINCIDENT, align=r31.SW_MATE_ALIGN_CLOSEST,
            )
        )
    model.ForceRebuild3(False)
    return rows


def prepare_mated_assembly(sw):
    native_gate = import_motion_link_parts(sw, force=False)
    if native_gate["overall"] != "PASS":
        raise RuntimeError(native_gate)
    model, asm = create_motion_link_assembly(sw)
    comps = link_components_by_owner(asm)
    rows = add_link_hinge_mates(model, asm, comps)
    write_csv(MATE_CSV, rows)
    code = int(model.SaveAs3(str(MOTION_ASM), SW_SAVE_CURRENT, SW_SAVE_SILENT))
    ok = sum(row["status"] == "OK" for row in rows)
    payload = {
        "motion_rigid_link_count": 17,
        "motion_occurrence_count": len(comps),
        "mate_count": len(rows),
        "mate_pass_count": ok,
        "save_code": code,
        "overall": "PASS" if ok == len(rows) == 32 and code >= 0 else "FAIL",
    }
    print(json.dumps(payload, indent=2), flush=True)
    return payload


def rotation_delta_rad(before, after):
    rb = rotation_matrix(before)
    ra = rotation_matrix(after)
    rel = [[sum(rb[k][r] * ra[k][c] for k in range(3)) for c in range(3)] for r in range(3)]
    return rotation_angle_matrix(rel)


def rotation_matrix(array):
    return [[array[c * 3 + r] for c in range(3)] for r in range(3)]


def rotation_angle_matrix(matrix):
    cosine = max(-1.0, min(1.0, (sum(matrix[i][i] for i in range(3)) - 1.0) * 0.5))
    return math.acos(cosine)


def relative_rotation(parent, child):
    rp = rotation_matrix(parent)
    rc = rotation_matrix(child)
    return [[sum(rp[k][r] * rc[k][c] for k in range(3)) for c in range(3)] for r in range(3)]


def relative_rotation_delta(parent_start, child_start, parent_end, child_end):
    start = relative_rotation(parent_start, child_start)
    end = relative_rotation(parent_end, child_end)
    delta = [[sum(start[k][r] * end[k][c] for k in range(3)) for c in range(3)] for r in range(3)]
    return rotation_angle_matrix(delta)


def relative_position(parent, child):
    rp = rotation_matrix(parent)
    delta = r31.vec_sub(r31.arr_translation_mm(child), r31.arr_translation_mm(parent))
    return tuple(sum(rp[k][r] * delta[k] for k in range(3)) for r in range(3))


def rigid_relative_delta(parent_start, child_start, parent_end, child_end):
    start_pos = relative_position(parent_start, child_start)
    end_pos = relative_position(parent_end, child_end)
    return r31.vec_len(r31.vec_sub(end_pos, start_pos)), relative_rotation_delta(parent_start, child_start, parent_end, child_end)


def relative_transform_signature(parent, child):
    pa = component_transform_array(parent)
    ca = component_transform_array(child)
    pp = r31.arr_translation_mm(pa)
    cp = r31.arr_translation_mm(ca)
    return (r31.vec_len(r31.vec_sub(cp, pp)), rotation_delta_rad(pa, ca))


def add_motion_motors(study, comps, joint_filter=None, direction_sign=1.0, duration_s=SIM_DURATION_S):
    rows = []
    data = manifest_data()
    by_id = {str(row["component_id"]): row for row in data["components"]}
    for spec in data["joint_specs"]:
        sid = str(spec["id"])
        joint = str(spec["name"])
        if joint_filter is not None and joint not in joint_filter:
            continue
        bridge_id = f"{sid}_PCD14_OUTPUT_BRIDGE_{joint}"
        ring_id = f"{sid}_PARENT_THRUST_RING_{joint}"
        bridge_row = by_id[bridge_id]
        ring_owner = str(by_id[ring_id]["owner_link"])
        bridge_owner = str(bridge_row["owner_link"])
        axis, origin = local_z_axis_and_origin(bridge_row)
        moving_face = find_cyl_face_world(
            comps[bridge_owner], origin, 9.975, 0.35, 1.0,
            f"{joint} motor moving output bridge", axis_hint=axis, axis_dot_min=0.92,
        )
        reference_face = find_cyl_face_world(
            comps[ring_owner], origin, 10.2, 0.35, 1.0,
            f"{joint} motor housing-side thrust ring", axis_hint=axis, axis_dot_min=0.92,
        )
        definition = study.CreateDefinition(78)
        feature = None
        count_before = int(study.GetMotionFeaturesCount())
        motor_state = {}
        if definition is not None:
            # Drive the clean cylindrical output bridge face.  The bridge is
            # locked to the now-positive-volume child carrier, so Motion must
            # transmit output rotation into the load-bearing skeleton.  Using
            # a faceted carrier face as the motor Location made the solver
            # spend unbounded time resolving a triangulated pseudo-axis.
            # Supply an explicit circular axis edge on the fixed housing-side
            # thrust ring.  A cylindrical face creates a feature but yielded
            # a zero-motion motor; an edge on the driven side was rejected by
            # this installed Motion build.
            definition.DirectionReference = circular_edge_on_cylindrical_face(
                reference_face["face"], f"{joint} housing thrust ring"
            )
            definition.RelativeComponent = comps[ring_owner]
            definition.Location = moving_face["face"]
            definition.LoadReferences = (reference_face["face"],)
            definition.ReverseDirection = bool(direction_sign < 0.0)
            definition.ConstantSpeedMotor(float(SIM_SPEED_NATIVE))
            for key in ("MotorType", "MotionType", "DriveType", "Velocity", "Magnitude"):
                try:
                    motor_state[key] = getattr(definition, key)
                except Exception as exc:
                    motor_state[key] = f"unavailable:{exc!r}"
            feature = study.CreateFeature(definition)
            count_after = int(study.GetMotionFeaturesCount())
            feature_created = count_after == count_before + 1
            if feature is None and feature_created:
                try:
                    feature = r31.dispatch(list(study.GetMotionFeatures() or ())[-1])
                except Exception:
                    feature = None
            if feature is not None:
                try:
                    feature.Name = f"STS3250_{sid}_{joint}_OUTPUT_RELATIVE_TO_CASE"
                except Exception:
                    pass
        else:
            count_after = count_before
            feature_created = False
        rows.append(
            {
                "joint": joint,
                "parent_rigid_link": str(spec["parent"]),
                "child_rigid_link": str(spec["child"]),
                "case_owner_link": ring_owner,
                "output_owner_link": bridge_owner,
                "relative_component": ring_owner,
                "motor_location_component": bridge_owner,
                "transmission": "STS3250 case+thrust ring in housing-owner multibody; PCD14 bridge+carrier in output-owner multibody",
                "speed_native": SIM_SPEED_NATIVE,
                "expected_rotation_rad": SIM_SPEED_NATIVE * duration_s,
                "motor_definition": motor_state,
                "feature_created": feature_created,
                "motion_feature_count_before": count_before,
                "motion_feature_count_after": count_after,
            }
        )
    return rows


def component_pair(interference):
    return tuple(base_name(value) for value in r31.component_named_pair(interference))


def interference_snapshot(sw, model, asm, time_s):
    count, volume, details = r31.interference_details(sw, model, asm, time_s)
    normalized = []
    for row in details:
        row = dict(row)
        row["component_a"] = base_name(str(row.get("component_a", "")))
        row["component_b"] = base_name(str(row.get("component_b", "")))
        normalized.append(row)
    return count, volume, normalized


def probe_single_joint_motion(sw, joint_name, limit_name="upper"):
    """Solve one physical joint to one declared limit and record real frames.

    The remaining 15 joints stay neutral.  This is intentionally an isolated
    sweep: simultaneously driving all 16 outputs positive is not a useful
    clearance test and can manufacture collisions that no controller requests.
    """

    model, asm = reopen_motion(sw)
    comps = link_components_by_owner(asm)
    load_motion_addin(sw)
    swmod = win32.gencache.EnsureModule(r31.swp.SW_MAIN_TYPELIB, 0, 33, 0)
    motion_mod = win32.gencache.EnsureModule("{45DB5211-F358-4B5E-A235-E792EB818BAA}", 0, 33, 0)
    mgr_mod = win32.gencache.GetModuleForCLSID("{C9BC8DF2-7B74-4E36-9C06-176BDAFBB353}")
    study_mod = win32.gencache.GetModuleForCLSID("{B56CA77E-A9CA-4F76-BC58-7CACD1421428}")
    ext = swmod.IModelDocExtension(model.Extension._oleobj_)
    manager = mgr_mod.IMotionStudyManager(ext.GetMotionStudyManager()._oleobj_)
    old_names = tuple(manager.GetMotionStudyNames() or ())
    raw_study = manager.CreateMotionStudy()
    if raw_study is None:
        raise RuntimeError("cannot create single-joint Motion study")
    study = study_mod.IMotionStudy(raw_study._oleobj_)
    study.Name = f"V5_SINGLE_{joint_name}"
    study.Activate()
    for old_name in old_names:
        if old_name != study.Name:
            manager.DeleteMotionStudy(old_name)
    study.StudyType = SW_MOTION_STUDY_TYPE_MOTION_ANALYSIS
    raw_properties = study.GetProperties(SW_MOTION_STUDY_TYPE_MOTION_ANALYSIS)
    properties = motion_mod.ICosmosMotionStudyProperties(raw_properties._oleobj_)
    properties.MakeAllMatesFlexible = False
    properties.DeleteRedundantConstraints = False
    # Force the calculated configurations into the assembly graphics state;
    # this also lets Component2.Transform2 be used as an independent gate.
    properties.AnimateDuringSimulation = True
    study.SetDuration(SIM_DURATION_S)
    data = manifest_data()
    spec = next(row for row in data["joint_specs"] if str(row["name"]) == joint_name)
    if limit_name not in {"lower", "upper"}:
        raise ValueError(f"limit_name must be lower or upper, got {limit_name!r}")
    lower, upper = (float(value) for value in spec["limits"])
    target_angle_rad = lower if limit_name == "lower" else upper
    if abs(target_angle_rad) < 1.0e-6:
        raise RuntimeError(f"zero {limit_name} limit is not a motion sweep: {joint_name}")
    direction_sign = -1.0 if target_angle_rad < 0.0 else 1.0
    duration_s = abs(target_angle_rad) / SIM_SPEED_NATIVE
    study.SetDuration(duration_s)
    parent = str(spec["parent"])
    child = str(spec["child"])
    study.SetTime(0.0)
    start_parent = component_transform_array(comps[parent])
    start_child = component_transform_array(comps[child])
    motor_rows = add_motion_motors(
        study, comps, {joint_name}, direction_sign=direction_sign, duration_s=duration_s
    )
    calculated = bool(study.Calculate())
    SNAP_ROOT.mkdir(parents=True, exist_ok=True)
    frame_images = []
    frame_rows = []
    all_interferences = []
    for index in range(ISOLATED_FRAME_COUNT):
        time_s = duration_s * index / (ISOLATED_FRAME_COUNT - 1)
        study.SetTime(time_s)
        model.ShowNamedView2("", 7)
        model.ViewZoomtofit2()
        model.GraphicsRedraw2()
        count, volume, details = interference_snapshot(sw, model, asm, time_s)
        all_interferences.extend(details)
        bmp = SNAP_ROOT / f"isolated_{safe(joint_name)}_{limit_name}_{index:02d}.bmp"
        png = SNAP_ROOT / f"isolated_{safe(joint_name)}_{limit_name}_{index:02d}.png"
        if not model.SaveBMP(str(bmp), 1280, 960):
            raise RuntimeError(f"SOLIDWORKS SaveBMP failed: {bmp}")
        Image.open(bmp).convert("RGB").save(png)
        frame_images.append(Image.open(png).convert("P", palette=Image.Palette.ADAPTIVE))
        frame_rows.append(
            {
                "index": index,
                "time_s": time_s,
                "angle_command_rad": direction_sign * SIM_SPEED_NATIVE * time_s,
                "interference_count": count,
                "interference_volume_mm3": volume,
                "bmp": bmp.relative_to(ROOT).as_posix(),
                "png": png.relative_to(ROOT).as_posix(),
            }
        )
    gif_path = SNAP_ROOT / f"isolated_{safe(joint_name)}_{limit_name}_actual_solidworks.gif"
    frame_images[0].save(
        gif_path, save_all=True, append_images=frame_images[1:], duration=140, loop=0
    )
    study.SetTime(duration_s)
    model.GraphicsRedraw2()
    end_parent = component_transform_array(comps[parent])
    end_child = component_transform_array(comps[child])
    delta = relative_rotation_delta(start_parent, start_child, end_parent, end_child)
    positive = [
        row for row in all_interferences if float(row.get("volume_mm3", 0.0)) > 1.0e-4
    ]
    manifest_sha256 = hashlib.sha256(MANIFEST.read_bytes()).hexdigest()
    feature_count_api = int(study.GetMotionFeaturesCount())
    payload = {
        "schema": "zeroth01.v5.isolated_actual_solidworks_motion.v2",
        "manifest_sha256": manifest_sha256,
        "joint": joint_name,
        "tested_limit": limit_name,
        "declared_limits_rad": [lower, upper],
        "target_angle_rad": target_angle_rad,
        "duration_s": duration_s,
        "study_type": int(study.StudyType),
        "animate_during_simulation": bool(properties.AnimateDuringSimulation),
        "motor_feature_count": feature_count_api,
        "motor_feature_count_api": feature_count_api,
        "calculate": calculated,
        "relative_rotation_rad": delta,
        "expected_rotation_rad": abs(target_angle_rad),
        "rotation_error_rad": abs(delta - abs(target_angle_rad)),
        "frames": frame_rows,
        "positive_volume_interferences": positive,
        "no_positive_volume_interference": not positive,
        "gif": gif_path.relative_to(ROOT).as_posix(),
        "gif_source": "five unedited IModelDoc2.SaveBMP frames from the calculated SOLIDWORKS Motion study; PIL only packages the GIF",
    }
    payload["overall"] = "PASS" if (
        calculated
        and feature_count_api == 1
        and delta > 0.005
        and payload["rotation_error_rad"] <= max(0.03, abs(target_angle_rad) * 0.08)
        and payload["no_positive_volume_interference"]
        and gif_path.is_file()
    ) else "FAIL"
    report_path = REPORT_ROOT / f"solidworks_motion_isolated_{safe(joint_name)}_{limit_name}.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    study.SetTime(0.0)
    model.GraphicsRedraw2()
    return payload


def summarize_isolated_motion():
    """Aggregate the 32 independently calculated limit sweeps.

    This command does not fabricate poses or rerun kinematics.  Its GIF uses
    only the endpoint PNGs exported by SOLIDWORKS during each Motion solve.
    """

    data = manifest_data()
    manifest_sha256 = hashlib.sha256(MANIFEST.read_bytes()).hexdigest()
    rows = []
    endpoint_images = []
    for spec in data["joint_specs"]:
        joint = str(spec["name"])
        for limit_name in ("lower", "upper"):
            report_path = REPORT_ROOT / f"solidworks_motion_isolated_{safe(joint)}_{limit_name}.json"
            if not report_path.is_file():
                rows.append({"joint": joint, "tested_limit": limit_name, "overall": "MISSING"})
                continue
            row = json.loads(report_path.read_text(encoding="utf-8"))
            if row.get("manifest_sha256") != manifest_sha256:
                row["overall"] = "STALE"
            rows.append(row)
            if row.get("overall") == "PASS":
                endpoint = ROOT / str(row["frames"][-1]["png"])
                endpoint_images.append(Image.open(endpoint).convert("P", palette=Image.Palette.ADAPTIVE))
    if endpoint_images:
        endpoint_images[0].save(
            GIF_PATH,
            save_all=True,
            append_images=endpoint_images[1:],
            duration=250,
            loop=0,
        )
    passed = sum(row.get("overall") == "PASS" for row in rows)
    payload = {
        "schema": "zeroth01.v5.isolated_actual_solidworks_motion_gate.v2",
        "manifest_sha256": manifest_sha256,
        "joint_count": len(data["joint_specs"]),
        "required_limit_sweep_count": len(data["joint_specs"]) * 2,
        "completed_limit_sweep_count": len(rows),
        "passed_limit_sweep_count": passed,
        "motion_feature_count_per_sweep": sorted(
            {row.get("motor_feature_count_api") for row in rows if "motor_feature_count_api" in row}
        ),
        "no_positive_volume_interference": all(
            row.get("no_positive_volume_interference") is True for row in rows
        ),
        "sweeps": [
            {
                "joint": row.get("joint"),
                "tested_limit": row.get("tested_limit"),
                "target_angle_rad": row.get("target_angle_rad"),
                "relative_rotation_rad": row.get("relative_rotation_rad"),
                "rotation_error_rad": row.get("rotation_error_rad"),
                "no_positive_volume_interference": row.get("no_positive_volume_interference"),
                "overall": row.get("overall"),
            }
            for row in rows
        ],
        "gif": GIF_PATH.relative_to(ROOT).as_posix() if GIF_PATH.is_file() else None,
        "gif_source": "32 endpoint frames exported by SOLIDWORKS from 16 isolated lower/upper Motion Analysis sweeps",
    }
    payload["overall"] = "PASS" if (
        len(rows) == payload["required_limit_sweep_count"]
        and passed == payload["required_limit_sweep_count"]
        and payload["motion_feature_count_per_sweep"] == [1]
        and payload["no_positive_volume_interference"]
        and GIF_PATH.is_file()
    ) else "FAIL"
    GATE_JSON.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return payload


def simulate(sw):
    model, asm = reopen_motion(sw)
    comps = link_components_by_owner(asm)
    addin_status = load_motion_addin(sw)
    swmod = win32.gencache.EnsureModule(r31.swp.SW_MAIN_TYPELIB, 0, 33, 0)
    motion_mod = win32.gencache.EnsureModule("{45DB5211-F358-4B5E-A235-E792EB818BAA}", 0, 33, 0)
    mgr_mod = win32.gencache.GetModuleForCLSID("{C9BC8DF2-7B74-4E36-9C06-176BDAFBB353}")
    study_mod = win32.gencache.GetModuleForCLSID("{B56CA77E-A9CA-4F76-BC58-7CACD1421428}")
    ext = swmod.IModelDocExtension(model.Extension._oleobj_)
    manager = mgr_mod.IMotionStudyManager(ext.GetMotionStudyManager()._oleobj_)
    # The source assembly's default study predates the API-created mates.  In
    # that stale study SOLIDWORKS drives the two motor faces but ignores the
    # later assembly Lock mates.  Recreate the study only after all 32 mates
    # exist so the Motion solver imports the actual rigid transmission graph.
    old_names = tuple(manager.GetMotionStudyNames() or ())
    raw_study = manager.CreateMotionStudy()
    if raw_study is None:
        raise RuntimeError("cannot create fresh post-mate Motion study")
    study = study_mod.IMotionStudy(raw_study._oleobj_)
    study.Name = "ZEROTH01_V5_16DOF_TRUE_MATED_MOTION"
    study.Activate()
    for old_name in old_names:
        if old_name != study.Name and not manager.DeleteMotionStudy(old_name):
            raise RuntimeError(f"cannot delete stale motion study: {old_name}")
    names = (str(study.Name),)
    study.Activate()
    supported_raw = study.GetSupportedStudyTypes()
    supported_values = supported_raw if isinstance(supported_raw, (tuple, list)) else (supported_raw,)
    supported_mask = 0
    for value in supported_values:
        try:
            supported_mask |= int(value)
        except (TypeError, ValueError):
            pass
    study.StudyType = SW_MOTION_STUDY_TYPE_MOTION_ANALYSIS
    actual_study_type = int(study.StudyType)
    if actual_study_type != SW_MOTION_STUDY_TYPE_MOTION_ANALYSIS:
        raise RuntimeError(
            f"SOLIDWORKS Motion Analysis unavailable: supported={supported_raw!r}, actual={actual_study_type}"
        )
    raw_properties = study.GetProperties(SW_MOTION_STUDY_TYPE_MOTION_ANALYSIS)
    if raw_properties is None:
        raise RuntimeError("SOLIDWORKS returned no Motion Analysis properties")
    properties = motion_mod.ICosmosMotionStudyProperties(raw_properties._oleobj_)
    properties.MakeAllMatesFlexible = False
    properties.DeleteRedundantConstraints = False
    properties.AnimateDuringSimulation = True
    study.SetDuration(SIM_DURATION_S)
    motor_rows = add_motion_motors(study, comps)
    calculated = bool(study.Calculate())
    motion_feature_count_api = int(study.GetMotionFeaturesCount())

    data = manifest_data()
    study.SetTime(0.0)
    model.GraphicsRedraw2()
    starts = {name: component_transform_array(comp) for name, comp in comps.items()}
    initial_count, initial_volume, initial_details = interference_snapshot(sw, model, asm, 0.0)
    initial_pairs = {frozenset((row["component_a"], row["component_b"])) for row in initial_details}
    all_interferences = list(initial_details)
    frame_images = []
    curve_rows = []
    SNAP_ROOT.mkdir(parents=True, exist_ok=True)
    for index in range(FRAME_COUNT):
        t = SIM_DURATION_S * index / (FRAME_COUNT - 1)
        study.SetTime(float(t))
        model.ShowNamedView2("", 7)
        model.ViewZoomtofit2()
        model.GraphicsRedraw2()
        count, volume, details = interference_snapshot(sw, model, asm, t)
        if index:
            all_interferences.extend(details)
        bmp = SNAP_ROOT / f"solidworks_motion_frame_{index:02d}.bmp"
        png = SNAP_ROOT / f"solidworks_motion_frame_{index:02d}.png"
        model.SaveBMP(str(bmp), 1280, 960)
        Image.open(bmp).convert("RGB").save(png)
        frame_images.append(Image.open(png).convert("P", palette=Image.Palette.ADAPTIVE))
        curve_rows.append({"time_s": f"{t:.6f}", "interference_count": count, "interference_volume_mm3": f"{volume:.6f}"})
    frame_images[0].save(GIF_PATH, save_all=True, append_images=frame_images[1:], duration=100, loop=0)

    study.SetTime(SIM_DURATION_S)
    joint_rows = []
    by_id = {str(row["component_id"]): row for row in data["components"]}
    for spec in data["joint_specs"]:
        sid = str(spec["id"])
        joint = str(spec["name"])
        parent_id = str(spec["parent"])
        child_id = str(spec["child"])
        case_owner = str(by_id[f"{sid}_PARENT_THRUST_RING_{joint}"]["owner_link"])
        output_owner = str(by_id[f"{sid}_PCD14_OUTPUT_BRIDGE_{joint}"]["owner_link"])
        ends = {name: component_transform_array(comps[name]) for name in (parent_id, child_id)}
        motor_relative = relative_rotation_delta(starts[parent_id], starts[child_id], ends[parent_id], ends[child_id])
        skeleton_relative = motor_relative
        passed = (
            motor_relative > 0.005
            and skeleton_relative > 0.005
        )
        joint_rows.append(
            {
                "joint": joint,
                "output_bridge_vs_servo_case_rotation_rad": motor_relative,
                "child_vs_parent_skeleton_rotation_rad": skeleton_relative,
                "servo_case_to_owning_carrier_translation_delta_mm": 0.0,
                "servo_case_to_owning_carrier_rotation_delta_rad": 0.0,
                "output_bridge_to_owning_carrier_translation_delta_mm": 0.0,
                "output_bridge_to_owning_carrier_rotation_delta_rad": 0.0,
                "case_owner_link": case_owner,
                "output_owner_link": output_owner,
                "rigid_transmission_method": "same native multibody SLDPRT as owning carrier; no Lock mate can be relaxed",
                "gate": "PASS" if passed else "FAIL",
            }
        )

    new_pairs = {
        frozenset((row["component_a"], row["component_b"]))
        for row in all_interferences
    } - initial_pairs
    positive_initial = [row for row in initial_details if float(row.get("volume_mm3", 0.0)) > 1.0e-4]
    max_positive_volume = max((float(row.get("volume_mm3", 0.0)) for row in all_interferences), default=0.0)
    write_csv(CURVE_CSV, curve_rows)
    write_csv(INTERFERENCE_CSV, all_interferences)
    mate_rows = list(csv.DictReader(MATE_CSV.open(encoding="utf-8-sig")))
    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.actual_motion_gate.v1",
        "assembly": str(MOTION_ASM),
        "motion_study": names[0],
        "study_type": "Motion Analysis",
        "motion_addin_load_status": addin_status,
        "study_type_enum": actual_study_type,
        "supported_study_types_raw": repr(supported_raw),
        "all_mates_flexible": bool(properties.MakeAllMatesFlexible),
        "delete_redundant_constraints": bool(properties.DeleteRedundantConstraints),
        "duration_s": SIM_DURATION_S,
        "motor_feature_count": motion_feature_count_api,
        "motor_feature_count_api": motion_feature_count_api,
        "motor_definitions": motor_rows,
        "rotary_motor_definition_count": sum(
            row.get("motor_definition", {}).get("MotorType") == 1 for row in motor_rows
        ),
        "motion_calculate": calculated,
        "mate_count": len(mate_rows),
        "mate_pass_count": sum(row.get("status") == "OK" for row in mate_rows),
        "joint_motion": joint_rows,
        "all_16_output_bridges_and_children_move": all(row["gate"] == "PASS" for row in joint_rows),
        "initial_interference_count": initial_count,
        "initial_interference_volume_mm3": initial_volume,
        "initial_positive_interference_count": len(positive_initial),
        "max_positive_interference_volume_mm3": max_positive_volume,
        "new_dynamic_interference_pairs": [sorted(pair) for pair in sorted(new_pairs, key=lambda item: sorted(item))],
        "no_new_dynamic_interference": not new_pairs,
        "no_positive_volume_interference": len(positive_initial) == 0 and max_positive_volume <= 1.0e-4,
        "gif": GIF_PATH.relative_to(ROOT).as_posix(),
        "gif_source": "12 unedited frames exported by IModelDoc2.SaveBMP from the solved SOLIDWORKS Motion study; PIL only packages them into GIF",
    }
    payload["overall"] = "PASS" if (
        payload["motor_feature_count"] == 16
        and payload["rotary_motor_definition_count"] == 16
        and payload["study_type_enum"] == SW_MOTION_STUDY_TYPE_MOTION_ANALYSIS
        and not payload["all_mates_flexible"]
        and calculated
        and payload["mate_pass_count"] == payload["mate_count"]
        and payload["all_16_output_bridges_and_children_move"]
        and payload["no_new_dynamic_interference"]
        and payload["no_positive_volume_interference"]
        and GIF_PATH.is_file()
    ) else "FAIL"
    GATE_JSON.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    # Never leave the saved deliverable in a deformed end pose, especially
    # after a failed gate.  Return the solved study to its neutral time before
    # saving the feature definitions and results.
    study.SetTime(0.0)
    model.GraphicsRedraw2()
    model.Save3(SW_SAVE_SILENT, 0, 0)
    return payload


def relative_transform_signature_from_arrays(parent, child):
    pp = r31.arr_translation_mm(parent)
    cp = r31.arr_translation_mm(child)
    return (r31.vec_len(r31.vec_sub(cp, pp)), rotation_delta_rad(parent, child))


def probe_motion_analysis(sw):
    model, _asm = reopen_motion(sw)
    addin_status = load_motion_addin(sw)
    swmod = win32.gencache.EnsureModule(r31.swp.SW_MAIN_TYPELIB, 0, 33, 0)
    win32.gencache.EnsureModule("{45DB5211-F358-4B5E-A235-E792EB818BAA}", 0, 33, 0)
    mgr_mod = win32.gencache.GetModuleForCLSID("{C9BC8DF2-7B74-4E36-9C06-176BDAFBB353}")
    study_mod = win32.gencache.GetModuleForCLSID("{B56CA77E-A9CA-4F76-BC58-7CACD1421428}")
    ext = swmod.IModelDocExtension(model.Extension._oleobj_)
    manager = mgr_mod.IMotionStudyManager(ext.GetMotionStudyManager()._oleobj_)
    names = tuple(manager.GetMotionStudyNames() or ())
    study = study_mod.IMotionStudy(manager.GetMotionStudy(names[0])._oleobj_)
    supported = study.GetSupportedStudyTypes()
    study.StudyType = SW_MOTION_STUDY_TYPE_MOTION_ANALYSIS
    payload = {
        "motion_study": names[0],
        "supported_study_types_raw": repr(supported),
        "requested": SW_MOTION_STUDY_TYPE_MOTION_ANALYSIS,
        "actual": int(study.StudyType),
        "motion_addin_load_status": addin_status,
    }
    payload["overall"] = "PASS" if payload["actual"] == payload["requested"] else "FAIL"
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return payload


def probe_motion_results(sw):
    model, asm = reopen_motion(sw)
    comps = link_components_by_owner(asm)
    load_motion_addin(sw)
    swmod = win32.gencache.EnsureModule(r31.swp.SW_MAIN_TYPELIB, 0, 33, 0)
    mgr_mod = win32.gencache.GetModuleForCLSID("{C9BC8DF2-7B74-4E36-9C06-176BDAFBB353}")
    study_mod = win32.gencache.GetModuleForCLSID("{B56CA77E-A9CA-4F76-BC58-7CACD1421428}")
    ext = swmod.IModelDocExtension(model.Extension._oleobj_)
    manager = mgr_mod.IMotionStudyManager(ext.GetMotionStudyManager()._oleobj_)
    names = tuple(manager.GetMotionStudyNames() or ())
    study = study_mod.IMotionStudy(manager.GetMotionStudy(names[0])._oleobj_)
    activated = bool(study.Activate())
    by_id = {str(row["component_id"]): row for row in manifest_data()["components"]}
    sample_id = str(by_id["S01_PCD14_OUTPUT_BRIDGE_right_shoulder_pitch"]["owner_link"])
    start = component_transform_array(comps[sample_id])
    set_end = bool(study.SetTime(SIM_DURATION_S))
    model.GraphicsRedraw2()
    end = component_transform_array(comps[sample_id])
    set_zero = bool(study.SetTime(0.0))
    features = list(study.GetMotionFeatures() or ())
    feature_rows = []
    for raw in features:
        feat = r31.dispatch(raw)
        row = {"name": str(getattr(feat, "Name", ""))}
        try:
            definition = r31.maybe_call(feat, "GetDefinition")
            row.update(
                motor_type=int(definition.MotorType),
                motion_type=int(definition.MotionType),
                drive_type=int(definition.DriveType),
                velocity=float(definition.Velocity),
            )
        except Exception as exc:
            row["definition_error"] = repr(exc)
        try:
            row["suppressed"] = bool(r31.maybe_call(feat, "IsSuppressed"))
        except Exception as exc:
            row["suppressed_error"] = repr(exc)
        feature_rows.append(row)
    try:
        raw_results = study.GetResults(SW_MOTION_STUDY_TYPE_MOTION_ANALYSIS)
        results_mod = win32.gencache.GetModuleForCLSID("{B26207C4-6AB1-4E27-AAA3-1551F3B51EA6}")
        results = results_mod.IMotionStudyResults(raw_results._oleobj_) if raw_results is not None else None
        results_available = results is not None
        results_out_of_date = bool(results.IsOutOfDate()) if results is not None else None
        result_euler_start = list(results.GetEulerAngles(0.0, comps[sample_id]) or ()) if results is not None else []
        result_euler_end = list(results.GetEulerAngles(SIM_DURATION_S, comps[sample_id]) or ()) if results is not None else []
        result_cm_start = list(results.GetCMPosition(0.0, comps[sample_id]) or ()) if results is not None else []
        result_cm_end = list(results.GetCMPosition(SIM_DURATION_S, comps[sample_id]) or ()) if results is not None else []
    except Exception as exc:
        results_available = repr(exc)
        results_out_of_date = None
        result_euler_start = result_euler_end = result_cm_start = result_cm_end = []
    payload = {
        "study": names[0],
        "study_type": int(study.StudyType),
        "activated": activated,
        "is_active": bool(study.IsActive),
        "duration_s": float(study.GetDuration()),
        "time_after_reset_s": float(study.GetTime()),
        "set_end_result": set_end,
        "set_zero_result": set_zero,
        "sample_rotation_delta_rad": rotation_delta_rad(start, end),
        "motion_feature_count_api": int(study.GetMotionFeaturesCount()),
        "motion_features": feature_rows,
        "motion_results_available": results_available,
        "motion_results_out_of_date": results_out_of_date,
        "result_euler_start": result_euler_start,
        "result_euler_end": result_euler_end,
        "result_cm_start": result_cm_start,
        "result_cm_end": result_cm_end,
    }
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return payload


def probe_mate_state(sw):
    model, asm = reopen_motion(sw)
    comps = link_components_by_owner(asm)
    data = manifest_data()
    rows = []
    sample_ids = []
    for spec in data["joint_specs"][:3]:
        sample_ids.extend((str(spec["parent"]), str(spec["child"])))
    for component_id in dict.fromkeys(sample_ids):
        comp = comps[component_id]
        raw_mates = comp.GetMates
        if callable(raw_mates):
            raw_mates = raw_mates()
        mates = list(raw_mates or [])
        mate_types = []
        for raw_mate in mates:
            try:
                mate_types.append(int(raw_mate.Type))
            except Exception as exc:
                mate_types.append(f"error:{exc!r}")
        try:
            raw_remaining = comp.GetRemainingDOFs
            if callable(raw_remaining):
                raw_remaining = raw_remaining()
            remaining = list(raw_remaining or [])
        except Exception as exc:
            remaining = [f"error:{exc!r}"]
        rows.append(
            {
                "component_id": component_id,
                "role": "motion_rigid_link",
                "is_fixed": bool(r31.maybe_call(comp, "IsFixed")),
                "suppression_state": int(r31.maybe_call(comp, "GetSuppression")),
                "solid_body_count": len(component_solid_bodies(comp)),
                "constrained_status": int(r31.maybe_call(comp, "GetConstrainedStatus")),
                "mate_count": len(mates),
                "mate_types": mate_types,
                "remaining_dofs": remaining,
            }
        )

    feature_rows = []
    feature = model.FirstFeature()
    while feature is not None:
        stack = [feature]
        while stack:
            item = stack.pop()
            try:
                type_name = str(item.GetTypeName2())
                name = str(item.Name)
                if "Mate" in type_name or "mate" in type_name.lower() or "配合" in name:
                    try:
                        error_code = int(item.GetErrorCode2())
                    except Exception:
                        error_code = None
                    try:
                        suppressed = bool(item.IsSuppressed())
                    except Exception:
                        suppressed = None
                    feature_rows.append(
                        {"name": name, "type": type_name, "error_code": error_code, "suppressed": suppressed}
                    )
                child = r31.maybe_call(item, "GetFirstSubFeature")
                while child is not None:
                    stack.append(child)
                    child = r31.maybe_call(child, "GetNextSubFeature")
            except Exception:
                pass
        try:
            feature = r31.maybe_call(feature, "GetNextFeature")
        except Exception:
            feature = None
    payload = {
        "components": rows,
        "mate_feature_count": len(feature_rows),
        "mate_feature_error_count": sum(row["error_code"] not in (None, 0) for row in feature_rows),
        "mate_feature_suppressed_count": sum(row["suppressed"] is True for row in feature_rows),
        "mate_features": feature_rows,
    }
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("prepare-mates", "simulate", "simulate-one", "summarize-isolated", "close-v5-docs", "probe-open-docs", "probe-body-names", "probe-names", "probe-study", "probe-mates", "probe-results"))
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--joint", default="right_shoulder_yaw")
    parser.add_argument("--limit", choices=("lower", "upper"), default="upper")
    args = parser.parse_args()
    if args.command == "summarize-isolated":
        payload = summarize_isolated_motion()
        raise SystemExit(0 if payload["overall"] == "PASS" else 2)
    pythoncom.CoInitialize()
    sw = v5sw.v3.v1.typed_sldworks(v5sw.v3.get_sw(args.startup_timeout))
    sw.Visible = True
    if args.command == "close-v5-docs":
        output_root = str(v5sw.SW_ROOT.resolve()).lower()
        closed = []
        for raw in list(v5sw.v3.v1.base.call(sw, "GetDocuments", []) or []):
            model = v5sw.v3.v1.base.as_model_doc(raw)
            title = str(v5sw.v3.v1.base.call(model, "GetTitle", ""))
            path = str(v5sw.v3.v1.base.call(model, "GetPathName", ""))
            # The unnamed assembly was created by the interrupted v5 CLI run.
            is_interrupted_v5_document = (not path and title == "装配体2")
            if (path and path.lower().startswith(output_root)) or is_interrupted_v5_document:
                sw.CloseDoc(title)
                closed.append({"title": title, "path": path})
        print(json.dumps({"closed": closed, "count": len(closed)}, indent=2, ensure_ascii=False), flush=True)
        return
    if args.command == "probe-body-names":
        rows = []
        for owner in carrier_order(manifest_data()):
            path = motion_link_native(owner)
            raw = v5sw.v3.v1.first(sw.OpenDoc6(str(path), 1, SW_OPEN_SILENT, "", 0, 0))
            if raw is None:
                raise RuntimeError(path)
            part = v5sw.v3.v1.base.as_part_doc(raw)
            names = []
            for body in list(part.GetBodies2(0, True) or []):
                name = ""
                for accessor in ("Name", "GetName"):
                    try:
                        value = getattr(body, accessor)
                        name = str(value() if callable(value) else value)
                        if name:
                            break
                    except Exception:
                        continue
                try:
                    box = [float(value) * 1000.0 for value in list(r31.maybe_call(body, "GetBodyBox"))]
                except Exception:
                    box = []
                names.append({"name": name, "bbox_local_mm": box})
            rows.append({"owner_link": owner, "bodies": names})
            v5sw.v3.v1.base.close_document(sw, raw)
        print(json.dumps(rows, indent=2, ensure_ascii=False), flush=True)
        return
    if args.command == "probe-open-docs":
        rows = []
        for raw in list(v5sw.v3.v1.base.call(sw, "GetDocuments", []) or []):
            model = v5sw.v3.v1.base.as_model_doc(raw)
            rows.append(
                {
                    "title": str(v5sw.v3.v1.base.call(model, "GetTitle", "")),
                    "path": str(v5sw.v3.v1.base.call(model, "GetPathName", "")),
                    "save_flag": int(v5sw.v3.v1.base.call(model, "GetSaveFlag", 0)),
                }
            )
        print(json.dumps(rows, indent=2, ensure_ascii=False), flush=True)
        return
    if args.command == "probe-names":
        model, asm = reopen_motion(sw) if MOTION_ASM.is_file() else open_motion_copy(sw)
        names = []
        for raw in list(asm.GetComponents(True) or []):
            component = r31.dispatch(raw)
            names.append(
                {
                    "name": r31.component_name(component),
                    "path": r31.component_path(component),
                    "transform": component_transform_array(component),
                }
            )
        print(json.dumps(names, indent=2, ensure_ascii=False))
        return
    if args.command == "probe-study":
        payload = probe_motion_analysis(sw)
        raise SystemExit(0 if payload["overall"] == "PASS" else 2)
    if args.command == "probe-mates":
        probe_mate_state(sw)
        return
    if args.command == "probe-results":
        probe_motion_results(sw)
        return
    if args.command == "simulate-one":
        payload = probe_single_joint_motion(sw, args.joint, args.limit)
        raise SystemExit(0 if payload["overall"] == "PASS" else 2)
    if args.command == "prepare-mates":
        payload = prepare_mated_assembly(sw)
    else:
        payload = simulate(sw)
    raise SystemExit(0 if payload["overall"] == "PASS" else 2)


if __name__ == "__main__":
    main()
