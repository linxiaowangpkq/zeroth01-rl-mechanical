"""Time exact OCCT distance and Boolean common for one v5 component pair."""

from __future__ import annotations

import argparse
import json
import time

from build123d import import_step

import diagnose_v5_offline_brep_interference as gate


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("component_a")
    parser.add_argument("component_b")
    parser.add_argument("--operation", choices=("distance", "common", "both"), default="both")
    args = parser.parse_args()
    data = json.loads(gate.MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    shapes = []
    for component_id in (args.component_a, args.component_b):
        row = by_id[component_id]
        started = time.perf_counter()
        shape = gate.transformed(
            import_step(gate.ROOT / str(row["source"])),
            row["transform_local_mm_to_world_mm"],
        )
        print(f"load {component_id}: {time.perf_counter() - started:.3f}s", flush=True)
        shapes.append(shape)
    if args.operation in {"distance", "both"}:
        started = time.perf_counter()
        value = gate.exact_distance(*shapes)
        print(f"distance_mm={value:.12g} seconds={time.perf_counter() - started:.3f}", flush=True)
    if args.operation in {"common", "both"}:
        started = time.perf_counter()
        value = gate.common_volume(*shapes)
        print(f"common_volume_mm3={value:.12g} seconds={time.perf_counter() - started:.3f}", flush=True)


if __name__ == "__main__":
    main()
