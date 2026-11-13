"""Command line entry point:  gtbench tune | run | export-demo | all"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="gtbench", description=__doc__)
    parser.add_argument("command", choices=["tune", "run", "export-demo", "all"])
    parser.add_argument("--out", type=Path, default=Path("results"), help="output directory")
    args = parser.parse_args(argv)

    t0 = time.time()
    if args.command in ("tune", "all"):
        from .tuning import tune_all

        print(json.dumps(tune_all(), indent=2))
    if args.command in ("run", "all"):
        from .benchmark import run_all
        from .report import write_report

        write_report(run_all(args.out), args.out)
        print(f"Results written to {args.out}/RESULTS.md")
    if args.command in ("export-demo", "all"):
        from .export import write_demo_data

        write_demo_data(Path("docs/demo-data.json"))
        print("Demo data written to docs/demo-data.json")
    print(f"Done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
