"""List small analytic cylindrical faces in the normalized exact STS3250."""

from __future__ import annotations

import json

from build123d import import_step
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

import diagnose_v5_offline_brep_interference as gate


def main() -> int:
    data = json.loads(gate.MANIFEST.read_text(encoding="utf-8"))
    source = gate.ROOT / str(next(
        row["source"] for row in data["components"] if row["role"] == "purchased_exact_sts3250"
    ))
    shape = import_step(source)
    rows = []
    explorer = TopExp_Explorer(shape.wrapped, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        adaptor = BRepAdaptor_Surface(face, True)
        if adaptor.GetType() == GeomAbs_Cylinder:
            cylinder = adaptor.Cylinder()
            radius = float(cylinder.Radius())
            axis = cylinder.Axis()
            direction = axis.Direction()
            location = axis.Location()
            # The case-mount through/threaded portions in the exact vendor
            # model are represented by the paired R0.8 and R1.9 cylindrical
            # faces.  Keep this diagnostic deliberately narrow so accessory
            # screw holes and output-horn features cannot be mistaken for the
            # four case-fastener axes.
            axial = abs(float(direction.Z())) >= 0.999
            target_radius = 0.75 <= radius <= 0.85 or 1.85 <= radius <= 1.95
            if axial and target_radius:
                v_first = float(adaptor.FirstVParameter())
                v_last = float(adaptor.LastVParameter())
                z_first = float(location.Z()) + float(direction.Z()) * v_first
                z_last = float(location.Z()) + float(direction.Z()) * v_last
                rows.append({
                    "radius_mm": radius,
                    "axis_origin_mm": [float(location.X()), float(location.Y()), float(location.Z())],
                    "axis_direction": [float(direction.X()), float(direction.Y()), float(direction.Z())],
                    "u_range": [float(adaptor.FirstUParameter()), float(adaptor.LastUParameter())],
                    "v_range": [v_first, v_last],
                    "z_face_span_mm": [min(z_first, z_last), max(z_first, z_last)],
                })
        explorer.Next()
    groups = {}
    for row in rows:
        key = (
            round(row["radius_mm"], 4),
            round(row["axis_origin_mm"][0], 3),
            round(row["axis_origin_mm"][1], 3),
            round(abs(row["axis_direction"][2]), 3),
        )
        group = groups.setdefault(
            key,
            {
                "radius_mm": key[0],
                "axis_xy_mm": [key[1], key[2]],
                "face_count": 0,
                "z_values_mm": [],
                "z_face_spans_mm": [],
            },
        )
        group["face_count"] += 1
        group["z_values_mm"].append(float(row["axis_origin_mm"][2]))
        group["z_face_spans_mm"].append(row["z_face_span_mm"])
    compact = []
    for group in groups.values():
        z_values = group.pop("z_values_mm")
        group["z_origin_range_mm"] = [min(z_values), max(z_values)]
        spans = group.pop("z_face_spans_mm")
        group["z_face_span_union_mm"] = [
            min(span[0] for span in spans),
            max(span[1] for span in spans),
        ]
        compact.append(group)
    compact.sort(key=lambda row: (row["radius_mm"], row["axis_xy_mm"]))
    per_solid = []
    for index, solid in enumerate(shape.solids(), start=1):
        box = solid.bounding_box()
        solid_axes = set()
        explorer = TopExp_Explorer(solid.wrapped, TopAbs_FACE)
        while explorer.More():
            face = TopoDS.Face_s(explorer.Current())
            adaptor = BRepAdaptor_Surface(face, True)
            if adaptor.GetType() == GeomAbs_Cylinder:
                cylinder = adaptor.Cylinder()
                radius = float(cylinder.Radius())
                axis = cylinder.Axis()
                direction = axis.Direction()
                location = axis.Location()
                if abs(float(direction.Z())) >= 0.999 and (
                    0.75 <= radius <= 0.85 or 1.85 <= radius <= 1.95
                ):
                    solid_axes.add(
                        (
                            round(radius, 4),
                            round(float(location.X()), 3),
                            round(float(location.Y()), 3),
                        )
                    )
            explorer.Next()
        per_solid.append(
            {
                "solid_index": index,
                "volume_mm3": round(float(solid.volume), 3),
                "bbox_min_mm": [round(float(value), 3) for value in box.min.to_tuple()],
                "bbox_max_mm": [round(float(value), 3) for value in box.max.to_tuple()],
                "target_axes": [list(axis) for axis in sorted(solid_axes)],
            }
        )
    per_solid.sort(key=lambda row: -row["volume_mm3"])
    print(
        json.dumps(
            {
                "source": str(source),
                "unique_small_cylinder_axes": compact,
                "manufacturing_solids_by_volume": per_solid,
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
