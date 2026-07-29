from __future__ import annotations

import argparse
import asyncio
import json
from datetime import datetime, timezone
from pathlib import Path

from kscale import K


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "source_assets" / "kscale_zbot_metadata.json"


async def fetch(robot_name: str) -> dict[str, object]:
    async with K() as api:
        robot_class = await api.get_robot_class(robot_name)
    return {
        "retrieved_utc": datetime.now(timezone.utc).isoformat(),
        "robot_name": robot_name,
        "robot_class": robot_class.model_dump(mode="json"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Snapshot public K-Scale robot metadata for the Zeroth/ZBot audit."
    )
    parser.add_argument("--robot", default="zbot")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output = args.output.resolve()
    if ROOT.drive.upper() != "E:" or output.drive.upper() != "E:":
        raise RuntimeError("Zeroth-01 artifacts must stay on the canonical E: workspace.")
    payload = asyncio.run(fetch(args.robot))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(output)


if __name__ == "__main__":
    main()
