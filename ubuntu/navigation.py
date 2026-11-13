#!/usr/bin/env python3
"""Use the frozen manual exam; translate only host paths and CLI arguments."""
import argparse
import os
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
REAL = ROOT / "baseline/TurtleBot-Real-2026-11-11"
SAMPLE = REAL / "runs/Real_103_2026-11-11_152732"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("preview", "navigate"))
    parser.add_argument("run", nargs="?", default=str(SAMPLE), help="recorded run directory or basename")
    parser.add_argument("--order-csv", help="CSV made from this same map")
    parser.add_argument("--expect-count", type=int, choices=range(1, 5))
    args = parser.parse_args()
    if args.mode == "navigate" and not sys.stdin.isatty():
        parser.error("Open an interactive Ubuntu desktop terminal; Enter requests a stop.")
    if args.mode == "navigate" and not shutil.which("rviz2"):
        parser.error("rviz2 was not found in ROS 2 Humble.")
    sys.path.insert(0, str(REAL / "scripts"))
    import manual_navigation as archived
    archived.PI = ROOT / "ubuntu/robot.sh"
    archived.RVIZ = shutil.which("rviz2")
    sys.argv = [__file__, "--recorded", "--exam", "--rviz-start", args.run]
    if args.mode == "preview":
        sys.argv.append("--preview")
    if args.order_csv:
        sys.argv.extend(("--order-csv", args.order_csv))
    if args.expect_count:
        sys.argv.extend(("--expect-count", str(args.expect_count)))
    os.environ["ROS_DOMAIN_ID"] = "3"
    return archived.main()


if __name__ == "__main__":
    sys.exit(main())
