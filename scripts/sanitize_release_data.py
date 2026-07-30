from __future__ import annotations

import csv
import json
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def relativize(value: str) -> str:
    if len(value) < 3 or not (
        value[0].isalpha() and value[1] == ":" and value[2] in "\\/"
    ):
        return value
    normalized = value.replace("\\", "/")
    marker = "/roboto_xw/reference/zeroth01/"
    marker_index = normalized.lower().find(marker)
    if marker_index >= 0:
        return normalized[marker_index + len(marker) :]
    fallback = normalized.lower().find("/zeroth01/")
    if fallback >= 0:
        return normalized[fallback + len("/zeroth01/") :]
    return f"EXTERNAL_ABSOLUTE_PATH_REMOVED/{Path(normalized).name}"


def sanitize_json_value(value: object) -> object:
    if isinstance(value, dict):
        return {key: sanitize_json_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [sanitize_json_value(item) for item in value]
    if isinstance(value, str):
        return relativize(value)
    return value


def sanitize_json(path: Path) -> bool:
    payload = json.loads(path.read_text(encoding="utf-8-sig"))
    sanitized = sanitize_json_value(payload)
    if sanitized == payload:
        return False
    path.write_text(
        json.dumps(sanitized, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return True


def sanitize_csv(path: Path) -> bool:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.reader(stream))
    sanitized = [[relativize(cell) for cell in row] for row in rows]
    if sanitized == rows:
        return False
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        csv.writer(stream).writerows(sanitized)
    return True


def sanitize_text(path: Path) -> bool:
    text = path.read_text(encoding="utf-8-sig")
    sanitized = re.sub(
        r"(?i)[A-Za-z]:\\(?:[^\\\r\n`]+\\)*?roboto_xw\\reference\\zeroth01\\",
        "",
        text,
    )
    if sanitized == text:
        return False
    path.write_text(sanitized, encoding="utf-8")
    return True


def main() -> None:
    changed: list[str] = []
    for path in sorted(ROOT.rglob("*")):
        if not path.is_file() or ".git" in path.parts:
            continue
        if path.suffix.lower() == ".json":
            if sanitize_json(path):
                changed.append(path.relative_to(ROOT).as_posix())
        elif path.suffix.lower() == ".csv":
            if sanitize_csv(path):
                changed.append(path.relative_to(ROOT).as_posix())
        elif path.suffix.lower() in {".md", ".txt"}:
            if sanitize_text(path):
                changed.append(path.relative_to(ROOT).as_posix())
    print(json.dumps({"changed_files": changed}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
