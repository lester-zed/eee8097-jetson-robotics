#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from experiments.recorder import load_jsonl_records
from experiments.statistics import summarize_records


DEFAULT_INPUT = Path("/workspace/logs/calibration/runs.jsonl")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize EEE8097 calibration and grasp experiment JSONL"
    )
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--output")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    records = load_jsonl_records(args.input)
    summary = summarize_records(records)
    rendered = json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True)
    print(rendered)
    if args.output:
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
        print(f"Report written: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
