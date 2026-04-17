#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any


DEFAULT_MODES = [
    "nonplanar_priority",
    "context_face_support",
    "context_normal_variation",
    "context_asymmetry",
    "context_hybrid",
]


def parse_summary(summary_path: Path) -> dict[str, Any]:
    parsed: dict[str, Any] = {}
    current_section: str | None = None
    with summary_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.rstrip("\n")
            if not line:
                continue
            if line.startswith("  ") and current_section is not None:
                key, value = line.strip().split(": ", 1)
                parsed.setdefault(current_section, {})[key] = int(value)
                continue
            if line.endswith(":"):
                current_section = line[:-1]
                parsed[current_section] = {}
                continue
            key, value = line.split(": ", 1)
            current_section = None
            parsed[key] = value
    return parsed


def parse_interesting_labels(csv_path: Path) -> tuple[Counter[str], int]:
    counts: Counter[str] = Counter()
    total = 0
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            counts[row["label"]] += 1
            total += 1
    return counts, total


def write_csv(output_path: Path, rows: list[dict[str, Any]], labels: list[str]) -> None:
    fieldnames = [
        "mode",
        "output_dir",
        "analyzed_voxels",
        "interesting_top_k",
        "average_gaussian_center_advantage",
        "average_shell_center_penalty",
        "average_morphology_filter_penalty",
        "average_occupied_face_ratio",
        "average_opposite_face_pair_ratio",
        "average_normal_variation",
        "average_occupancy_asymmetry",
        "average_planar_context_penalty",
        "average_corner_context_bonus",
        "delta_corner_like_fraction_vs_baseline",
        "delta_planar_fraction_vs_baseline",
    ]
    for prefix in ("label", "topk"):
        for label in labels:
            fieldnames.append(f"{prefix}_{label}_count")
            fieldnames.append(f"{prefix}_{label}_fraction")

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            flat = {key: row[key] for key in fieldnames if key in row}
            for prefix in ("label", "topk"):
                counts = row[f"{prefix}_counts"]
                total = max(sum(counts.values()), 1)
                for label in labels:
                    count = counts.get(label, 0)
                    flat[f"{prefix}_{label}_count"] = count
                    flat[f"{prefix}_{label}_fraction"] = count / total
            writer.writerow(flat)


def write_markdown(output_path: Path, rows: list[dict[str, Any]], baseline_mode: str) -> None:
    baseline = next(row for row in rows if row["mode"] == baseline_mode)
    baseline_topk = baseline["topk_counts"]
    baseline_total = max(sum(baseline_topk.values()), 1)

    feature_modes = [row for row in rows if row["mode"] in {
        "context_face_support",
        "context_normal_variation",
        "context_asymmetry",
    }]
    feature_best = max(
        feature_modes,
        key=lambda row: (
            -row["delta_planar_fraction_vs_baseline"],
            row["delta_corner_like_fraction_vs_baseline"],
        ),
    )
    hybrid = next(row for row in rows if row["mode"] == "context_hybrid")

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# Context Refinement Summary\n\n")
        handle.write("## Baseline\n\n")
        handle.write(
            f"- `{baseline_mode}`: corner_like={baseline_topk.get('corner_like', 0)}/{baseline_total} "
            f"({baseline_topk.get('corner_like', 0) / baseline_total:.3f}), "
            f"planar={baseline_topk.get('planar', 0)}/{baseline_total} "
            f"({baseline_topk.get('planar', 0) / baseline_total:.3f})\n\n"
        )

        handle.write("## Mode Comparison\n\n")
        for row in rows:
            topk = row["topk_counts"]
            total = max(sum(topk.values()), 1)
            handle.write(
                f"- `{row['mode']}`: corner_like={topk.get('corner_like', 0)}/{total} "
                f"({topk.get('corner_like', 0) / total:.3f}), "
                f"planar={topk.get('planar', 0)}/{total} "
                f"({topk.get('planar', 0) / total:.3f}), "
                f"delta_corner={row['delta_corner_like_fraction_vs_baseline']:+.3f}, "
                f"delta_planar={row['delta_planar_fraction_vs_baseline']:+.3f}\n"
            )

        handle.write("\n## Best Feature Ablation\n\n")
        handle.write(
            f"- `{feature_best['mode']}` reduced planar fraction by "
            f"{-feature_best['delta_planar_fraction_vs_baseline']:.3f}p and changed corner_like fraction by "
            f"{feature_best['delta_corner_like_fraction_vs_baseline']:+.3f}p versus baseline.\n"
        )

        handle.write("\n## Hybrid\n\n")
        hybrid_topk = hybrid["topk_counts"]
        hybrid_total = max(sum(hybrid_topk.values()), 1)
        handle.write(
            f"- `context_hybrid`: corner_like={hybrid_topk.get('corner_like', 0)}/{hybrid_total} "
            f"({hybrid_topk.get('corner_like', 0) / hybrid_total:.3f}), "
            f"planar={hybrid_topk.get('planar', 0)}/{hybrid_total} "
            f"({hybrid_topk.get('planar', 0) / hybrid_total:.3f})\n"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Run context-refinement ranking modes with the C++ analyzer.")
    parser.add_argument("--input", required=True)
    parser.add_argument("--output-root", default="results/context_refinement")
    parser.add_argument("--voxel-size", type=float, default=1.0)
    parser.add_argument("--min-points-per-voxel", type=int, default=15)
    parser.add_argument("--shape-exponent", type=float, default=1.0)
    parser.add_argument("--axis-scale-quantile", type=float, default=0.95)
    parser.add_argument("--top-k", type=int, default=200)
    parser.add_argument("--modes", default=",".join(DEFAULT_MODES))
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    input_path = (repo_root / args.input).resolve() if not Path(args.input).is_absolute() else Path(args.input)
    output_root = (repo_root / args.output_root).resolve()
    runs_root = output_root / "runs"
    runs_root.mkdir(parents=True, exist_ok=True)

    analyzer = repo_root / "build" / "ges_voxel_mapping_cpp" / "voxel_morphology_analyzer"
    if not analyzer.exists():
        subprocess.run([str(repo_root / "scripts" / "build_offline.sh")], check=True)

    modes = [item.strip() for item in args.modes.split(",") if item.strip()]
    rows: list[dict[str, Any]] = []
    labels: set[str] = set()

    for mode in modes:
        run_output_dir = runs_root / mode
        command = [
            str(analyzer),
            "--input",
            str(input_path),
            "--mode",
            "single",
            "--output",
            str(run_output_dir),
            "--voxel-size",
            str(args.voxel_size),
            "--min-points-per-voxel",
            str(args.min_points_per_voxel),
            "--shape-exponent",
            str(args.shape_exponent),
            "--axis-scale-quantile",
            str(args.axis_scale_quantile),
            "--ranking-mode",
            mode,
            "--save-top-k",
            str(args.top_k),
            "--export-interesting-voxels",
            "false",
            "--export-top-k-pcd",
            "0",
        ]
        print(" ".join(command), flush=True)
        subprocess.run(command, check=True)

        summary = parse_summary(run_output_dir / "summary.txt")
        topk_counts, topk_total = parse_interesting_labels(run_output_dir / "interesting_voxels.csv")
        label_counts = Counter(summary.get("label_counts", {}))
        labels.update(label_counts.keys())
        labels.update(topk_counts.keys())

        rows.append(
            {
                "mode": mode,
                "output_dir": str(run_output_dir),
                "analyzed_voxels": int(summary["analyzed_voxels"]),
                "interesting_top_k": topk_total,
                "average_gaussian_center_advantage": float(summary["average_gaussian_center_advantage"]),
                "average_shell_center_penalty": float(summary["average_shell_center_penalty"]),
                "average_morphology_filter_penalty": float(summary["average_morphology_filter_penalty"]),
                "average_occupied_face_ratio": float(summary["average_occupied_face_ratio"]),
                "average_opposite_face_pair_ratio": float(summary["average_opposite_face_pair_ratio"]),
                "average_normal_variation": float(summary["average_normal_variation"]),
                "average_occupancy_asymmetry": float(summary["average_occupancy_asymmetry"]),
                "average_planar_context_penalty": float(summary["average_planar_context_penalty"]),
                "average_corner_context_bonus": float(summary["average_corner_context_bonus"]),
                "label_counts": dict(label_counts),
                "topk_counts": dict(topk_counts),
            }
        )

    baseline = next(row for row in rows if row["mode"] == "nonplanar_priority")
    baseline_total = max(sum(baseline["topk_counts"].values()), 1)
    baseline_corner_fraction = baseline["topk_counts"].get("corner_like", 0) / baseline_total
    baseline_planar_fraction = baseline["topk_counts"].get("planar", 0) / baseline_total

    for row in rows:
        total = max(sum(row["topk_counts"].values()), 1)
        corner_fraction = row["topk_counts"].get("corner_like", 0) / total
        planar_fraction = row["topk_counts"].get("planar", 0) / total
        row["delta_corner_like_fraction_vs_baseline"] = corner_fraction - baseline_corner_fraction
        row["delta_planar_fraction_vs_baseline"] = planar_fraction - baseline_planar_fraction

    rows.sort(
        key=lambda row: (
            row["delta_corner_like_fraction_vs_baseline"],
            -row["delta_planar_fraction_vs_baseline"],
            row["topk_counts"].get("corner_like", 0) - row["topk_counts"].get("planar", 0),
        ),
        reverse=True,
    )

    write_csv(output_root / "comparison.csv", rows, sorted(labels))
    write_markdown(output_root / "summary.md", rows, "nonplanar_priority")
    print(f"Wrote {output_root / 'comparison.csv'}")
    print(f"Wrote {output_root / 'summary.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
