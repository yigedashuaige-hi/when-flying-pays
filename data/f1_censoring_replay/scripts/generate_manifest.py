#!/usr/bin/env python3
"""Freeze all Phase F1 censoring-sensitivity evidence with SHA256."""

import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "SHA256_MANIFEST.txt"
OUTER = ROOT / "SHA256_MANIFEST.sha256"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    excluded = {MANIFEST.resolve(), OUTER.resolve()}
    paths = sorted(
        path for path in ROOT.rglob("*")
        if path.is_file()
        and path.resolve() not in excluded
        and "__pycache__" not in path.parts
        and path.suffix != ".pyc"
    )
    with MANIFEST.open("w") as stream:
        for path in paths:
            stream.write("%s  %s\n" % (sha256(path), path.relative_to(ROOT)))
    OUTER.write_text("%s  SHA256_MANIFEST.txt\n" % sha256(MANIFEST))
    print("manifest_entries=%d" % len(paths))


if __name__ == "__main__":
    main()
