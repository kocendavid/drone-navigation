"""Analyze a timestamped recording or incident; never connects to the drone."""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.diagnostics import analyze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.recording.suffix == ".json":
        records = json.loads(args.recording.read_text(encoding="utf-8"))["records"]
    else:
        with args.recording.open(encoding="utf-8") as stream:
            records = [json.loads(line) for line in stream if line.strip()]
    summary = analyze(records)
    content = json.dumps(summary, allow_nan=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(content, encoding="utf-8")
    else:
        print(content, end="")


if __name__ == "__main__":
    main()
