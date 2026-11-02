"""Fetch the reviewed Fusion AHRS subset at an immutable upstream revision."""

import argparse
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen


REVISION = "a8d7224f36a0ec82345ef49a3db50e65f8d3bab8"
FILES = (
    "Fusion/FusionAhrs.c",
    "Fusion/FusionAhrs.h",
    "Fusion/FusionMath.h",
    "Fusion/FusionConvention.c",
    "Fusion/FusionConvention.h",
    "Fusion/FusionConfig.h",
    "Fusion/FusionInline.h",
    "LICENSE.md",
)


def main():
    destination = Path(__file__).resolve().parents[1] / "third_party" / "Fusion"
    # Fetch everything before changing any local files.
    downloaded = {}
    for name in FILES:
        url = f"https://raw.githubusercontent.com/xioTechnologies/Fusion/{REVISION}/{name}"
        with urlopen(url, timeout=30) as response:
            downloaded[name] = response.read()
    hashes = {}
    for name, data in downloaded.items():
        output = destination / name
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    manifest = {
        "repository": "https://github.com/xioTechnologies/Fusion",
        "revision": REVISION,
        "sha256": hashes,
    }
    (destination / "UPSTREAM.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(f"Vendored {len(FILES)} unchanged upstream files at {REVISION}")


def verify():
    destination = Path(__file__).resolve().parents[1] / "third_party" / "Fusion"
    manifest = json.loads((destination / "UPSTREAM.json").read_text(encoding="utf-8"))
    if manifest["revision"] != REVISION or set(manifest["sha256"]) != set(FILES):
        raise SystemExit("Unexpected Fusion revision or file list")
    for name, expected in manifest["sha256"].items():
        actual = hashlib.sha256((destination / name).read_bytes()).hexdigest()
        if actual != expected:
            raise SystemExit(f"Upstream file changed: {name}")
    print(f"Verified {len(FILES)} unchanged Fusion files at {REVISION}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify", action="store_true", help="check local files without network access")
    arguments = parser.parse_args()
    if arguments.verify:
        verify()
    else:
        main()
