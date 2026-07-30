from __future__ import annotations

import argparse
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TEXT_SUFFIXES = {".csv", ".json", ".log", ".md", ".txt"}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Remove machine-local source-root prefixes from release data."
    )
    parser.add_argument("--source-root", type=Path, required=True)
    args = parser.parse_args()

    source = str(args.source_root.resolve())
    slash = source.replace("\\", "/")
    escaped = source.replace("\\", "\\\\")
    replacements = (
        (escaped + "\\\\", ""),
        (escaped, "."),
        (source + "\\", ""),
        (source, "."),
        (slash + "/", ""),
        (slash, "."),
    )
    changed: list[str] = []
    for base in (
        ROOT / "reports",
        ROOT / "generated" / "config",
        ROOT / "generated" / "solidworks" / "portable_flat_round_v1",
    ):
        for path in sorted(base.rglob("*")):
            if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
                continue
            text = path.read_text(encoding="utf-8-sig")
            updated = text
            for old, new in replacements:
                updated = updated.replace(old, new)
            if updated != text:
                path.write_text(updated, encoding="utf-8", newline="\n")
                changed.append(path.relative_to(ROOT).as_posix())
    print(f"SANITIZED={len(changed)}")
    for relative in changed:
        print(relative)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
