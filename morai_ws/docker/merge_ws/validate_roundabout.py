#!/usr/bin/env python3
"""Validate user-supplied CSV geometry without ROS or vehicle commands."""
import argparse
from pathlib import Path
import sys
import yaml

ws = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ws / "src/control/purepursuit_mgeo/src"))
sys.path.insert(0, str(ws / "src/mission/roundabout_mission/src"))
from purepursuit_mgeo.path import load_mgeo_path
from roundabout_mission.regions import load_regions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allow-empty", action="store_true")
    args = parser.parse_args()
    directory = ws / "config/roundabout"
    with (directory / "defaults.yaml").open(encoding="utf-8") as stream:
        defaults = yaml.safe_load(stream)
    try:
        points = load_mgeo_path(str(ws / "data/routes/2026_molit_comp_global_path.txt"))
        regions = load_regions(str(directory / "regions.csv"), points, defaults, load_mgeo_path)
        if not regions and not args.allow_empty:
            raise ValueError("회전교차로 구간 CSV가 비어 있습니다. 구간을 지정한 뒤 켜세요.")
    except (ValueError, OSError) as exc:
        parser.exit(2, str(exc) + "\n")
    print("ROUNDABOUT_REGIONS=%d (explicit CSV only; no inferred curves)" % len(regions))
    for name, engine in regions:
        print(name, "start_s=", engine.m["approach"], "end_s=", engine.m["exit"])


if __name__ == "__main__":
    main()
