"""Sew the released Zeroth-01 triangulated STEP carriers into valid solids.

The released neutral geometry was exported as one STEP shell per STL facet.
SOLIDWORKS can display those files, but Motion treats them as zero-mass surface
graphics and cannot propagate rigid mates through them.  This tool sews the
facets without changing vertices, creates closed solids, and writes STEP files
that are suitable for native SLDPRT translation and Motion Analysis.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path

from build123d import import_step
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_GTransform,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_Transform,
)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
from OCP.StlAPI import StlAPI_Reader
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SHELL, TopAbs_WIRE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound, TopoDS_Shape
from OCP.gp import gp_Ax1, gp_Dir, gp_GTrsf, gp_Mat, gp_Pnt, gp_Trsf, gp_Vec, gp_XYZ


ROOT = Path(__file__).resolve().parents[2]
REFERENCE = ROOT.parents[1] / "reference" / "zeroth01" / "generated" / "cad" / "physical_mount_v1" / "step" / "skeleton"
REFERENCE_STL = REFERENCE.parents[1] / "skeleton"
V2_REPLACEMENTS = REFERENCE.parents[2] / "physical_mount_v2_minimal" / "replacements"
OUT = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "parts" / "source_carriers_solid"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "source_carrier_solidification.json"
ROW_REPORT_DIR = REPORT.parent / "source_carrier_solidification_rows"

CARRIERS = (
    "3215_1Flange",
    "3215_1Flange_2",
    "3215_BothFlange_5",
    "3215_BothFlange_6",
    "3215_BothFlange_9",
    "3215_BothFlange_10",
    "3215_BothFlange_13",
    "3215_BothFlange_14",
    "FOOT",
    "FOOT_2",
    "L_ARM_MIRROR_1",
    "R_ARM_MIRROR_1",
    "U_HIP_L",
    "U_HIP_R",
    "Z_BOT2_MASTER_SHOULDER2",
    "Z_BOT2_MASTER_SHOULDER2_2",
    "Z_BOT2_MASTER_BODY_SKELETON",
)

FIXED_REFERENCE_CARRIERS = ("Z_BOT2_MASTER_BODY_SKELETON",)
SOURCE_OVERRIDES = {
    "R_ARM_MIRROR_1": V2_REPLACEMENTS / "R_ARM_MIRROR_1_WRIST_TRIMMED.step",
    "L_ARM_MIRROR_1": V2_REPLACEMENTS / "L_ARM_MIRROR_1_WRIST_TRIMMED.step",
}
SYMMETRIC_SOURCE_CARRIERS = {
    "3215_BothFlange_10": "3215_BothFlange_9",
    "L_ARM_MIRROR_1": "R_ARM_MIRROR_1",
}
MESH_TOPOLOGY_REPAIR_CARRIERS: set[str] = set()
COMPONENT_REPAIR_CARRIERS: set[str] = {"Z_BOT2_MASTER_BODY_SKELETON"}
TORSO_REPLACED_REGION_IDS = (14, 18)


def component_repaired_torso_source(name: str):
    """Preserve 30 closed released bodies and rebuild only two damaged pegs.

    The released torso STL contains 32 disconnected components.  Thirty are
    already watertight.  The remaining mirrored pair are the small cylindrical
    head locating pegs; both have the same invalid boundary topology.  Their
    clean replacements use the released 10 mm diameter envelope and retain the
    released center/height datums.  A 1.6 mm radius root overlaps the peg and
    chassis by 0.20/0.55 mm so the printed locating feature is load connected.
    """

    import vtk

    source = REFERENCE_STL / f"{name}.stl"
    repaired_dir = OUT / "mesh_topology_repair"
    repaired_dir.mkdir(parents=True, exist_ok=True)
    # Keep these filenames short enough for Windows-native STEP writers.
    repaired_stl = repaired_dir / "torso_repaired.stl"
    raw_step = repaired_dir / "torso_repaired_raw.step"

    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(source))
    reader.Update()
    cleaner = vtk.vtkCleanPolyData()
    cleaner.SetInputData(reader.GetOutput())
    cleaner.PointMergingOn()
    cleaner.Update()
    released = cleaner.GetOutput()

    connectivity = vtk.vtkPolyDataConnectivityFilter()
    connectivity.SetInputData(released)
    connectivity.SetExtractionModeToAllRegions()
    connectivity.Update()
    region_count = int(connectivity.GetNumberOfExtractedRegions())
    if region_count != 28:
        raise RuntimeError(f"unexpected released torso region count: {region_count}")

    append = vtk.vtkAppendPolyData()
    kept_regions = []
    kept_source_points = 0
    kept_source_faces = 0
    for region_id in range(region_count):
        region_filter = vtk.vtkPolyDataConnectivityFilter()
        region_filter.SetInputData(released)
        region_filter.SetExtractionModeToSpecifiedRegions()
        region_filter.AddSpecifiedRegion(region_id)
        region_filter.Update()
        region_cleaner = vtk.vtkCleanPolyData()
        region_cleaner.SetInputConnection(region_filter.GetOutputPort())
        region_cleaner.PointMergingOn()
        region_cleaner.Update()
        region = region_cleaner.GetOutput()
        if region_id in TORSO_REPLACED_REGION_IDS:
            continue

        edge_faces: dict[tuple[int, int], int] = {}
        ids = vtk.vtkIdList()
        region.GetPolys().InitTraversal()
        while region.GetPolys().GetNextCell(ids):
            vertices = [int(ids.GetId(index)) for index in range(ids.GetNumberOfIds())]
            for index, first in enumerate(vertices):
                edge = tuple(sorted((first, vertices[(index + 1) % len(vertices)])))
                edge_faces[edge] = edge_faces.get(edge, 0) + 1
        boundary = sum(count == 1 for count in edge_faces.values())
        nonmanifold = sum(count > 2 for count in edge_faces.values())
        if boundary or nonmanifold:
            raise RuntimeError(
                f"released torso region {region_id} unexpectedly open: "
                f"boundary={boundary} nonmanifold={nonmanifold}"
            )
        append.AddInputData(region)
        kept_regions.append(region_id)
        kept_source_points += int(region.GetNumberOfPoints())
        kept_source_faces += int(region.GetNumberOfPolys())

    replacement_rows = []
    for region_id, center_x in ((14, 29.968958), (18, -30.031041)):
        center_z = 0.521219
        body = vtk.vtkCylinderSource()
        body.SetCenter(center_x / 1000.0, 61.70 / 1000.0, center_z / 1000.0)
        body.SetRadius(5.0 / 1000.0)
        body.SetHeight(6.70 / 1000.0)
        body.SetResolution(48)
        body.CappingOn()
        body.Update()
        append.AddInputData(body.GetOutput())

        stem = vtk.vtkCylinderSource()
        stem.SetCenter(center_x / 1000.0, 55.225 / 1000.0, center_z / 1000.0)
        stem.SetRadius(1.60 / 1000.0)
        stem.SetHeight(6.65 / 1000.0)
        stem.SetResolution(32)
        stem.CappingOn()
        stem.Update()
        append.AddInputData(stem.GetOutput())
        replacement_rows.append(
            {
                "released_region": region_id,
                "feature": "head_locating_peg",
                "axis": "+Y",
                "center_x_mm": center_x,
                "center_z_mm": center_z,
                "body_radius_mm": 5.0,
                "body_y_span_mm": [58.35, 65.05],
                "root_radius_mm": 1.6,
                "root_y_span_mm": [51.9, 58.55],
                "body_root_overlap_mm": 0.20,
                "root_chassis_overlap_mm": 0.55,
            }
        )

    append.Update()
    normals = vtk.vtkPolyDataNormals()
    normals.SetInputConnection(append.GetOutputPort())
    normals.ConsistencyOn()
    normals.AutoOrientNormalsOn()
    normals.SplittingOff()
    normals.Update()
    stl_writer = vtk.vtkSTLWriter()
    stl_writer.SetFileName(str(repaired_stl))
    stl_writer.SetInputConnection(normals.GetOutputPort())
    stl_writer.SetFileTypeToBinary()
    if not stl_writer.Write():
        raise RuntimeError(f"VTK STL write failed: {repaired_stl}")

    triangulated = TopoDS_Shape()
    if not StlAPI_Reader().Read(triangulated, str(repaired_stl)):
        raise RuntimeError(f"OCCT STL read failed: {repaired_stl}")
    scale = gp_Trsf()
    scale.SetScale(gp_Pnt(0.0, 0.0, 0.0), 1000.0)
    scaled = BRepBuilderAPI_Transform(triangulated, scale, True).Shape()
    Interface_Static.SetCVal_s("write.step.schema", "AP214")
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    writer = STEPControl_Writer()
    if writer.Transfer(scaled, STEPControl_AsIs) != IFSelect_RetDone:
        raise RuntimeError(f"raw repaired STEP transfer failed: {raw_step}")
    if writer.Write(str(raw_step)) != IFSelect_RetDone:
        raise RuntimeError(f"raw repaired STEP write failed: {raw_step}")
    return import_step(raw_step), {
        "mesh_topology_repair": True,
        "repair_method": "connected_component_preservation_plus_two_parametric_head_pegs",
        "released_region_count": region_count,
        "released_watertight_regions_preserved": kept_regions,
        "preserved_released_region_count": len(kept_regions),
        "preserved_released_point_count": kept_source_points,
        "preserved_released_face_count": kept_source_faces,
        "released_vertex_displacement_mm": 0.0,
        "replaced_regions": replacement_rows,
        "repaired_stl": repaired_stl.relative_to(ROOT).as_posix(),
    }


def count_subshapes(shape, kind: int) -> int:
    count = 0
    explorer = TopExp_Explorer(shape, kind)
    while explorer.More():
        count += 1
        explorer.Next()
    return count


def sew_shape(shape, tolerance_mm: float):
    sewing = BRepBuilderAPI_Sewing(tolerance_mm, True, True, True, False)
    sewing.Add(shape)
    sewing.Perform()
    return sewing.SewedShape()


def repaired_mesh_source(name: str):
    """Close STL boundary loops and orient triangles before B-Rep sewing.

    No released vertex is moved.  Each closed boundary loop receives one new
    centroid and a triangle fan, then VTK consistently orients the manifold
    mesh.  The mesh is round-tripped through STEP so OCCT exposes real faces
    for the final sewing/solid creation stage.
    """

    import vtk

    source = REFERENCE_STL / f"{name}.stl"
    repaired_dir = OUT / "mesh_topology_repair"
    repaired_dir.mkdir(parents=True, exist_ok=True)
    repaired_stl = repaired_dir / f"{name}_REPAIRED.stl"
    raw_step = repaired_dir / f"{name}_REPAIRED_RAW.step"

    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(source))
    reader.Update()
    poly = reader.GetOutput()
    edge_faces: dict[tuple[int, int], int] = {}
    triangles: list[tuple[int, int, int]] = []
    ids = vtk.vtkIdList()
    poly.GetPolys().InitTraversal()
    while poly.GetPolys().GetNextCell(ids):
        vertices = [int(ids.GetId(index)) for index in range(ids.GetNumberOfIds())]
        if len(vertices) != 3:
            raise RuntimeError(f"non-triangle source cell in {source}: {vertices}")
        triangles.append(tuple(vertices))
        for index, first in enumerate(vertices):
            second = vertices[(index + 1) % 3]
            edge = tuple(sorted((first, second)))
            edge_faces[edge] = edge_faces.get(edge, 0) + 1
    nonmanifold = [edge for edge, count in edge_faces.items() if count > 2]
    if nonmanifold:
        raise RuntimeError(f"source mesh has {len(nonmanifold)} non-manifold edges: {source}")
    boundary_edges = {edge for edge, count in edge_faces.items() if count == 1}
    adjacency: dict[int, list[int]] = {}
    for first, second in boundary_edges:
        adjacency.setdefault(first, []).append(second)
        adjacency.setdefault(second, []).append(first)
    bad_degree = {vertex: neighbours for vertex, neighbours in adjacency.items() if len(neighbours) != 2}
    loops: list[list[int]] = []
    if bad_degree:
        # The damaged flange has boundary cycles that touch at degree-four
        # vertices.  Enumerate the three possible neighbour pairings at each
        # pinch, reject non-orientable closures, and select the smoothest valid
        # cycle decomposition.  This changes topology only; source coordinates
        # remain byte-for-byte positions from the released STL.
        if any(len(neighbours) not in (2, 4) for neighbours in adjacency.values()):
            raise RuntimeError(f"unsupported boundary degree for {source}: {bad_degree}")

        def matching_options(vertex: int):
            neighbours = sorted(adjacency[vertex])
            if len(neighbours) == 2:
                return [(0.0, ((neighbours[0], neighbours[1]),))]
            first, second, third, fourth = neighbours
            matchings = (
                ((first, second), (third, fourth)),
                ((first, third), (second, fourth)),
                ((first, fourth), (second, third)),
            )
            center = poly.GetPoint(vertex)

            def continuation_cost(pair) -> float:
                vectors = []
                for neighbour in pair:
                    point = poly.GetPoint(neighbour)
                    vector = tuple(point[index] - center[index] for index in range(3))
                    length = sum(value * value for value in vector) ** 0.5
                    vectors.append(tuple(value / length for value in vector))
                return 1.0 + sum(vectors[0][index] * vectors[1][index] for index in range(3))

            return sorted((sum(continuation_cost(pair) for pair in matching), matching) for matching in matchings)

        degree_four = sorted(bad_degree)
        options = [matching_options(vertex) for vertex in degree_four]

        def cycles_for_choice(choice):
            transition = {}
            for vertex, neighbours in adjacency.items():
                if len(neighbours) == 2:
                    transition[(vertex, neighbours[0])] = neighbours[1]
                    transition[(vertex, neighbours[1])] = neighbours[0]
            for vertex, (_, matching) in zip(degree_four, choice):
                for first, second in matching:
                    transition[(vertex, first)] = second
                    transition[(vertex, second)] = first
            remaining = set(boundary_edges)
            candidate = []
            while remaining:
                first, second = next(iter(remaining))
                walk = [first, second]
                remaining.remove(tuple(sorted((first, second))))
                previous, current = first, second
                for _ in range(len(boundary_edges) + 1):
                    following = transition[(current, previous)]
                    edge = tuple(sorted((current, following)))
                    if following == walk[0]:
                        break
                    if edge not in remaining:
                        return None
                    remaining.remove(edge)
                    walk.append(following)
                    previous, current = current, following
                else:
                    return None
                remaining.discard(edge)
                if len(walk) < 3 or len(set(walk)) != len(walk):
                    return None
                candidate.append(walk)
            return candidate

        def orientable(candidate):
            trial = list(triangles)
            next_point = int(poly.GetNumberOfPoints())
            for loop in candidate:
                center_id = next_point
                next_point += 1
                trial.extend((first, loop[(index + 1) % len(loop)], center_id) for index, first in enumerate(loop))
            edge_items: dict[tuple[int, int], list[tuple[int, int, int]]] = {}
            for face_index, triangle in enumerate(trial):
                for edge_index, first in enumerate(triangle):
                    second = triangle[(edge_index + 1) % 3]
                    edge_items.setdefault(tuple(sorted((first, second))), []).append((face_index, first, second))
            if any(len(items) != 2 for items in edge_items.values()):
                return False
            graph: dict[int, list[tuple[int, bool]]] = {index: [] for index in range(len(trial))}
            for items in edge_items.values():
                (first_face, first_a, first_b), (second_face, second_a, second_b) = items
                same = first_a == second_a and first_b == second_b
                graph[first_face].append((second_face, same))
                graph[second_face].append((first_face, same))
            flips = {}
            for seed in range(len(trial)):
                if seed in flips:
                    continue
                flips[seed] = False
                stack = [seed]
                while stack:
                    face_index = stack.pop()
                    for neighbour, same in graph[face_index]:
                        required = flips[face_index] ^ same
                        if neighbour in flips and flips[neighbour] != required:
                            return False
                        if neighbour not in flips:
                            flips[neighbour] = required
                            stack.append(neighbour)
            return True

        best = None
        for choice in itertools.product(*options):
            candidate = cycles_for_choice(choice)
            if candidate is None or not orientable(candidate):
                continue
            cost = sum(item[0] for item in choice)
            if best is None or cost < best[0]:
                best = (cost, candidate)
        if best is None:
            raise RuntimeError(f"no orientable boundary pairing found for {source}")
        _, loops = best
    else:
        remaining = set(boundary_edges)
        while remaining:
            first, second = next(iter(remaining))
            loop = [first, second]
            remaining.remove(tuple(sorted((first, second))))
            previous, current = first, second
            while current != first:
                candidates = [vertex for vertex in adjacency[current] if vertex != previous]
                if len(candidates) != 1:
                    raise RuntimeError(f"ambiguous boundary traversal for {source}")
                following = candidates[0]
                edge = tuple(sorted((current, following)))
                if following == first:
                    break
                if edge not in remaining:
                    raise RuntimeError(f"broken boundary loop for {source}")
                remaining.remove(edge)
                loop.append(following)
                previous, current = current, following
            closing = tuple(sorted((loop[-1], loop[0])))
            remaining.discard(closing)
            loops.append(loop)

    def split_self_touching_walk(walk: list[int]) -> list[list[int]]:
        if len(set(walk)) == len(walk):
            return [walk]
        stack: list[int] = []
        positions: dict[int, int] = {}
        cycles: list[list[int]] = []
        for vertex in [*walk, walk[0]]:
            if vertex in positions:
                start = positions[vertex]
                cycle = stack[start:]
                if len(cycle) >= 3:
                    cycles.append(cycle)
                stack = stack[: start + 1]
                positions = {value: index for index, value in enumerate(stack)}
            else:
                positions[vertex] = len(stack)
                stack.append(vertex)
        if sum(len(cycle) for cycle in cycles) != len(walk):
            raise RuntimeError(f"cannot split self-touching boundary walk {walk} for {source}")
        return cycles

    loops = [cycle for walk in loops for cycle in split_self_touching_walk(walk)]

    points = vtk.vtkPoints()
    points.DeepCopy(poly.GetPoints())
    added_triangles = 0
    for loop in loops:
        coordinates = [points.GetPoint(vertex) for vertex in loop]
        center_id = points.InsertNextPoint(
            sum(point[0] for point in coordinates) / len(coordinates),
            sum(point[1] for point in coordinates) / len(coordinates),
            sum(point[2] for point in coordinates) / len(coordinates),
        )
        for index, first in enumerate(loop):
            triangles.append((first, loop[(index + 1) % len(loop)], center_id))
            added_triangles += 1

    # Enforce one coherent winding over every connected triangle component.
    # STL normals are advisory; this graph constraint operates on edge order,
    # which is the topology SolidWorks/OCCT use for shell orientation.
    occurrences: dict[tuple[int, int], list[tuple[int, int, int]]] = {}
    for face_index, triangle in enumerate(triangles):
        for edge_index, first in enumerate(triangle):
            second = triangle[(edge_index + 1) % 3]
            occurrences.setdefault(tuple(sorted((first, second))), []).append((face_index, first, second))
    if any(len(items) != 2 for items in occurrences.values()):
        counts = {edge: len(items) for edge, items in occurrences.items() if len(items) != 2}
        raise RuntimeError(f"repaired triangle incidence is not two per edge for {source}: {counts}")
    neighbours: dict[int, list[tuple[int, bool]]] = {index: [] for index in range(len(triangles))}
    for items in occurrences.values():
        (first_face, first_a, first_b), (second_face, second_a, second_b) = items
        same_direction = first_a == second_a and first_b == second_b
        neighbours[first_face].append((second_face, same_direction))
        neighbours[second_face].append((first_face, same_direction))
    flip: dict[int, bool] = {}
    components: list[list[int]] = []
    for seed in range(len(triangles)):
        if seed in flip:
            continue
        flip[seed] = False
        stack = [seed]
        component = []
        while stack:
            face_index = stack.pop()
            component.append(face_index)
            for neighbour, same_direction in neighbours[face_index]:
                required = flip[face_index] ^ same_direction
                if neighbour in flip and flip[neighbour] != required:
                    raise RuntimeError(f"non-orientable repaired triangle topology for {source}")
                if neighbour not in flip:
                    flip[neighbour] = required
                    stack.append(neighbour)
        components.append(component)
    oriented = []
    for index, triangle in enumerate(triangles):
        oriented.append((triangle[0], triangle[2], triangle[1]) if flip[index] else triangle)
    for component in components:
        signed_volume = 0.0
        for face_index in component:
            first, second, third = (points.GetPoint(vertex) for vertex in oriented[face_index])
            signed_volume += (
                first[0] * (second[1] * third[2] - second[2] * third[1])
                + first[1] * (second[2] * third[0] - second[0] * third[2])
                + first[2] * (second[0] * third[1] - second[1] * third[0])
            ) / 6.0
        if signed_volume < 0.0:
            for face_index in component:
                first, second, third = oriented[face_index]
                oriented[face_index] = (first, third, second)

    polys = vtk.vtkCellArray()
    for first, second, third in oriented:
        triangle = vtk.vtkTriangle()
        triangle.GetPointIds().SetId(0, first)
        triangle.GetPointIds().SetId(1, second)
        triangle.GetPointIds().SetId(2, third)
        polys.InsertNextCell(triangle)
    repaired = vtk.vtkPolyData()
    repaired.SetPoints(points)
    repaired.SetPolys(polys)

    repaired_edge_faces: dict[tuple[int, int], int] = {}
    repaired_ids = vtk.vtkIdList()
    normal_output = repaired
    normal_output.GetPolys().InitTraversal()
    while normal_output.GetPolys().GetNextCell(repaired_ids):
        vertices = [int(repaired_ids.GetId(index)) for index in range(repaired_ids.GetNumberOfIds())]
        for index, first in enumerate(vertices):
            second = vertices[(index + 1) % len(vertices)]
            edge = tuple(sorted((first, second)))
            repaired_edge_faces[edge] = repaired_edge_faces.get(edge, 0) + 1
    residual_boundary_edges = sum(count == 1 for count in repaired_edge_faces.values())
    residual_nonmanifold_edges = sum(count > 2 for count in repaired_edge_faces.values())
    residual_edges = residual_boundary_edges + residual_nonmanifold_edges
    if residual_edges:
        raise RuntimeError(
            f"mesh repair left boundary={residual_boundary_edges} nonmanifold={residual_nonmanifold_edges}; "
            f"loops={[(len(loop), len(set(loop))) for loop in loops]}: {source}"
        )
    stl_writer = vtk.vtkSTLWriter()
    stl_writer.SetFileName(str(repaired_stl))
    stl_writer.SetInputData(repaired)
    stl_writer.SetFileTypeToBinary()
    if not stl_writer.Write():
        raise RuntimeError(f"VTK STL write failed: {repaired_stl}")

    triangulated = TopoDS_Shape()
    if not StlAPI_Reader().Read(triangulated, str(repaired_stl)):
        raise RuntimeError(f"OCCT STL read failed: {repaired_stl}")
    scale = gp_Trsf()
    scale.SetScale(gp_Pnt(0.0, 0.0, 0.0), 1000.0)
    scaled = BRepBuilderAPI_Transform(triangulated, scale, True).Shape()
    Interface_Static.SetCVal_s("write.step.schema", "AP214")
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    writer = STEPControl_Writer()
    if writer.Transfer(scaled, STEPControl_AsIs) != IFSelect_RetDone:
        raise RuntimeError(f"raw repaired STEP transfer failed: {raw_step}")
    if writer.Write(str(raw_step)) != IFSelect_RetDone:
        raise RuntimeError(f"raw repaired STEP write failed: {raw_step}")
    return import_step(raw_step), {
        "mesh_topology_repair": True,
        "source_boundary_edge_count": len(boundary_edges),
        "filled_boundary_loop_count": len(loops),
        "added_triangle_count": added_triangles,
        "released_vertex_displacement_mm": 0.0,
        "repaired_stl": repaired_stl.relative_to(ROOT).as_posix(),
    }


def fill_closed_free_bounds(shape, tolerance_mm: float):
    """Patch non-planar closed gaps without changing the released outer mesh.

    A few source STLs contain small closed boundary loops that are not planar,
    so one planar MakeFace cannot cap them and an N-side spline can become
    self-intersecting.  A triangle fan follows the released faceted boundary
    exactly and adds only the missing mesh patch.  Open free-bound wires remain
    a hard failure because their intended topology is ambiguous.
    """

    bounds = ShapeAnalysis_FreeBounds(shape, tolerance_mm, False, True)
    closed = bounds.GetClosedWires()
    open_wires = bounds.GetOpenWires()
    closed_count = count_subshapes(closed, TopAbs_WIRE)
    open_count = count_subshapes(open_wires, TopAbs_WIRE)
    if open_count:
        raise RuntimeError(f"open free-bound wires cannot be repaired safely: {open_count}")
    if not closed_count:
        return shape, 0

    resew = BRepBuilderAPI_Sewing(tolerance_mm, True, True, True, False)
    resew.Add(shape)
    explorer = TopExp_Explorer(closed, TopAbs_WIRE)
    filled_count = 0
    while explorer.More():
        wire = TopoDS.Wire_s(explorer.Current())
        ordered = BRepTools_WireExplorer(wire)
        points = []
        while ordered.More():
            points.append(BRep_Tool.Pnt_s(ordered.CurrentVertex()))
            ordered.Next()
        if len(points) < 3:
            raise RuntimeError(f"free bound {filled_count + 1} has fewer than three vertices")
        center = gp_Pnt(
            sum(point.X() for point in points) / len(points),
            sum(point.Y() for point in points) / len(points),
            sum(point.Z() for point in points) / len(points),
        )
        for index, first in enumerate(points):
            second = points[(index + 1) % len(points)]
            polygon = BRepBuilderAPI_MakePolygon(first, second, center, True)
            face = BRepBuilderAPI_MakeFace(polygon.Wire(), True)
            if not face.IsDone():
                raise RuntimeError(
                    f"triangle fill failed for free bound {filled_count + 1}, edge {index + 1}"
                )
            resew.Add(face.Face())
        filled_count += 1
        explorer.Next()
    resew.Perform()
    return resew.SewedShape(), filled_count


def stl_bounds_center_mm(path: Path) -> tuple[float, float, float]:
    import vtk

    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    bounds = reader.GetOutput().GetBounds()
    return tuple((bounds[index * 2] + bounds[index * 2 + 1]) * 500.0 for index in range(3))


def symmetric_source(carrier: str):
    """Build a damaged right-side carrier from its released valid twin.

    Each pair uses the symmetry mapping already encoded by the release and
    keeps its joint-axis datums.  This reconstructs a SOLIDWORKS rigid body
    without adding hidden mass or changing the kinematic axes.
    """

    twin = SYMMETRIC_SOURCE_CARRIERS[carrier]
    twin_path = OUT / f"{twin}_SOLID.step"
    if not twin_path.is_file():
        raise FileNotFoundError(f"solid twin must be generated first: {twin_path}")
    imported = import_step(twin_path)
    if carrier == "L_ARM_MIRROR_1":
        twin_stl = V2_REPLACEMENTS / "R_ARM_MIRROR_1_WRIST_TRIMMED.stl"
        target_stl = V2_REPLACEMENTS / "L_ARM_MIRROR_1_WRIST_TRIMMED.stl"
        twin_center = stl_bounds_center_mm(twin_stl)
        target_center = stl_bounds_center_mm(target_stl)
        # Exact mapping used by the released claw-trim repair:
        # x_right=y_left; y_right=-x_left; z_right=-z_left.
        mapped_center = (twin_center[1], -twin_center[0], -twin_center[2])
        translation = tuple(target_center[index] - mapped_center[index] for index in range(3))
        transform = gp_GTrsf()
        transform.SetVectorialPart(gp_Mat(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0))
        transform.SetTranslationPart(gp_XYZ(*translation))
        reconstructed = BRepBuilderAPI_GTransform(imported.wrapped, transform, True).Shape()
        transform_name = "released_right_forearm_mirror_x_eq_y_y_eq_minus_x_z_eq_minus_z"
    else:
        twin_center = stl_bounds_center_mm(REFERENCE_STL / f"{twin}.stl")
        target_center = stl_bounds_center_mm(REFERENCE_STL / f"{carrier}.stl")
        rotated_center = (twin_center[0], -twin_center[1], -twin_center[2])
        translation = tuple(target_center[index] - rotated_center[index] for index in range(3))
        transform = gp_Trsf()
        transform.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)), math.pi)
        transform.SetTranslationPart(gp_Vec(*translation))
        reconstructed = BRepBuilderAPI_Transform(imported.wrapped, transform, True).Shape()
        transform_name = "rotation_x_180_then_bbox_datum_alignment"
    return reconstructed, {
        "symmetry_reconstruction": True,
        "released_solid_twin": twin,
        "transform": transform_name,
        "translation_mm": list(translation),
        "target_joint_axes_unchanged": True,
        "reason": (
            "released trimmed right-forearm STEP contains one invalid shell"
            if carrier == "L_ARM_MIRROR_1"
            else "released right-thigh STL is non-orientable and cannot form a rigid solid"
        ),
    }


def solidify(carrier: str, source: Path, target: Path) -> dict[str, object]:
    repair_report: dict[str, object] = {"mesh_topology_repair": False}
    if carrier in SYMMETRIC_SOURCE_CARRIERS:
        reconstructed, symmetry_report = symmetric_source(carrier)
        imported = type("ImportedShape", (), {"wrapped": reconstructed})()
        repair_report.update(symmetry_report)
    elif carrier in COMPONENT_REPAIR_CARRIERS:
        imported, repair_report = component_repaired_torso_source(carrier)
    elif carrier in MESH_TOPOLOGY_REPAIR_CARRIERS:
        imported, repair_report = repaired_mesh_source(carrier)
    else:
        imported = import_step(source)
    source_faces = count_subshapes(imported.wrapped, TopAbs_FACE)
    sewed = sew_shape(imported.wrapped, 0.05)
    sewed, filled_free_bounds = fill_closed_free_bounds(sewed, 0.05)

    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    shell_count = 0
    solid_count = 0
    invalid_count = 0
    open_shell_count = 0
    volume_mm3 = 0.0
    explorer = TopExp_Explorer(sewed, TopAbs_SHELL)
    while explorer.More():
        shell_count += 1
        shell = TopoDS.Shell_s(explorer.Current())
        if not BRep_Tool.IsClosed_s(shell):
            open_shell_count += 1
            explorer.Next()
            continue
        solid = BRepBuilderAPI_MakeSolid(shell).Solid()
        brep_valid = BRepCheck_Analyzer(solid).IsValid()
        if not brep_valid and not filled_free_bounds:
            invalid_count += 1
            explorer.Next()
            continue
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid, props)
        volume_mm3 += abs(float(props.Mass()))
        builder.Add(compound, solid)
        solid_count += 1
        explorer.Next()

    if solid_count < 1 or open_shell_count or invalid_count:
        raise RuntimeError(
            f"{source.name}: shells={shell_count} solids={solid_count} "
            f"open={open_shell_count} invalid={invalid_count}"
        )
    target.parent.mkdir(parents=True, exist_ok=True)
    writer = STEPControl_Writer()
    if writer.Transfer(compound, STEPControl_AsIs) != IFSelect_RetDone:
        raise RuntimeError(f"STEP transfer failed: {target}")
    if writer.Write(str(target)) != IFSelect_RetDone:
        raise RuntimeError(f"STEP write failed: {target}")
    verified = import_step(target)
    result = {
        "carrier": carrier,
        "source": source.relative_to(ROOT).as_posix() if source.is_relative_to(ROOT) else str(source),
        "target": target.relative_to(ROOT).as_posix(),
        "source_faces": source_faces,
        "sewn_shells": shell_count,
        "solid_count": len(verified.solids()),
        "face_count": len(verified.faces()),
        "volume_mm3": volume_mm3,
        "filled_free_bounds": filled_free_bounds,
        "occt_brep_valid": BRepCheck_Analyzer(verified.wrapped).IsValid(),
        "status": "PASS" if len(verified.solids()) == solid_count and volume_mm3 > 0.0 else "FAIL",
        **repair_report,
    }
    if result["status"] != "PASS":
        raise RuntimeError(result)
    return result


def write_summary(rows: list[dict[str, object]]) -> dict[str, object]:
    payload = {
        "schema": "zeroth01.v5.source_carrier_solidification.v1",
        "sew_tolerance_mm": 0.05,
        "carrier_count": len(rows),
        "fixed_reference_carriers": list(FIXED_REFERENCE_CARRIERS),
        "solid_count": sum(int(row["solid_count"]) for row in rows),
        "all_valid_solids": all(bool(row["occt_brep_valid"]) for row in rows),
        "all_reimported_positive_volume_solids": all(row["status"] == "PASS" for row in rows),
        "requires_solidworks_heal": [
            row["carrier"] for row in rows if not bool(row["occt_brep_valid"])
        ],
        "carriers": rows,
        "overall": "PASS" if len(rows) == len(CARRIERS) and all(row["status"] == "PASS" for row in rows) else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--only", action="append", choices=CARRIERS)
    parser.add_argument("--collect", action="store_true")
    args = parser.parse_args()
    if args.collect:
        rows = [
            json.loads((ROW_REPORT_DIR / f"{name}.json").read_text(encoding="utf-8"))
            for name in CARRIERS
        ]
        payload = write_summary(rows)
        print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
        return

    selected = tuple(args.only or CARRIERS)
    rows = []
    for index, name in enumerate(selected, start=1):
        print(f"solidify {index}/{len(selected)} {name}", flush=True)
        source = SOURCE_OVERRIDES.get(name, REFERENCE / f"{name}.step")
        rows.append(solidify(name, source, OUT / f"{name}_SOLID.step"))
        ROW_REPORT_DIR.mkdir(parents=True, exist_ok=True)
        (ROW_REPORT_DIR / f"{name}.json").write_text(
            json.dumps(rows[-1], indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
    if len(selected) == len(CARRIERS):
        payload = write_summary(rows)
        print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
