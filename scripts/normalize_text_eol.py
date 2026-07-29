from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TEXT_SUFFIXES = {
    ".csv",
    ".json",
    ".md",
    ".py",
    ".txt",
    ".urdf",
    ".xml",
    ".yaml",
    ".yml",
}
TEXT_NAMES = {".gitattributes", ".gitignore"}


def main() -> int:
    changed = 0
    for path in sorted(ROOT.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(ROOT)
        if ".git" in relative.parts:
            continue
        if path.suffix.lower() not in TEXT_SUFFIXES and path.name not in TEXT_NAMES:
            continue
        raw = path.read_bytes()
        text = raw.decode("utf-8-sig")
        normalized = text.replace("\r\n", "\n").replace("\r", "\n").encode("utf-8")
        if normalized != raw:
            path.write_bytes(normalized)
            changed += 1
    print(f"NORMALIZED_TEXT_FILES={changed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
