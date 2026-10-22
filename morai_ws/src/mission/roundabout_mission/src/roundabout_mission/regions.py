"""Explicit CSV regions only. Curvature is never used to infer a roundabout."""
import csv
from pathlib import Path
from .core import Decision


COLUMNS = ["region_id", "start_index", "end_index", "stop_pose_index", "entry_index",
           "conflict_start_index", "conflict_end_index", "merge_complete_index",
           "circulating_path_file", "circulating_conflict_start_index",
           "circulating_conflict_end_index"]


def load_regions(csv_file, points, template, load_path):
    result, ids, spans = [], set(), []
    base = Path(csv_file).resolve().parent
    with open(csv_file, newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != COLUMNS:
            raise ValueError("roundabout CSV columns must be: " + ",".join(COLUMNS))
        for row in reader:
            if None in row:
                raise ValueError("too many roundabout CSV columns")
            if not any(value and value.strip() for value in row.values()):
                continue
            if any(not row.get(key, "").strip() for key in COLUMNS):
                raise ValueError("roundabout region has unfilled columns; geometry is never guessed")
            name = row["region_id"].strip()
            if name in ids:
                raise ValueError("duplicate roundabout region: " + name)
            ids.add(name)
            cfg = dict(template)
            mapping = dict(approach="start_index", stop_line="stop_pose_index", entry="entry_index",
                           conflict_start="conflict_start_index", conflict_end="conflict_end_index",
                           merge_complete="merge_complete_index", exit="end_index")
            cfg["mission_indices"] = {key: int(row[column]) for key, column in mapping.items()}
            cfg["entry_decision_stop_index"] = int(row["stop_pose_index"])
            cfg["circulating_conflict_indices"] = [int(row["circulating_conflict_start_index"]),
                                                   int(row["circulating_conflict_end_index"])]
            reference = Path(row["circulating_path_file"])
            if not reference.is_absolute():
                reference = base / reference
            try:
                reference.resolve().relative_to(base)
            except ValueError:
                raise ValueError("circulating reference must be inside the roundabout config directory")
            circulating = load_path(str(reference))
            cfg["roi_map"] = [min(p.x for p in circulating)-5, max(p.x for p in circulating)+5,
                              min(p.y for p in circulating)-5, max(p.y for p in circulating)+5]
            engine = Decision(points, circulating, cfg)
            span = (engine.m["approach"], engine.m["exit"])
            if any(max(span[0], a) < min(span[1], b) for a, b in spans):
                raise ValueError("roundabout regions overlap")
            spans.append(span)
            result.append((name, engine))
    return sorted(result, key=lambda item: item[1].m["approach"])
