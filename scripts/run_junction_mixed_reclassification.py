#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any


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


def parse_topk_counts(csv_path: Path) -> tuple[Counter[str], int]:
    counts: Counter[str] = Counter()
    total = 0
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            counts[row["label"]] += 1
            total += 1
    return counts, total


def run_mode(
    analyzer: Path,
    input_path: Path,
    run_output_dir: Path,
    args: argparse.Namespace,
    ranking_mode: str,
    enable_relabel: bool,
    relabel_mode: str,
) -> dict[str, Any]:
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
        ranking_mode,
        "--enable-junction-mixed-relabel",
        "true" if enable_relabel else "false",
        "--junction-mixed-relabel-mode",
        relabel_mode,
        "--junction-mixed-min-neighbor-count",
        str(args.junction_mixed_min_neighbor_count),
        "--junction-mixed-min-cluster-count",
        str(args.junction_mixed_min_cluster_count),
        "--junction-mixed-min-score",
        str(args.junction_mixed_min_score),
        "--junction-mixed-min-orientation-dispersion",
        str(args.junction_mixed_min_orientation_dispersion),
        "--junction-mixed-max-dominant-fraction",
        str(args.junction_mixed_max_dominant_fraction),
        "--junction-mixed-min-occupancy-asymmetry",
        str(args.junction_mixed_min_occupancy_asymmetry),
        "--junction-mixed-min-normal-variation",
        str(args.junction_mixed_min_normal_variation),
        "--junction-mixed-max-opposite-face-pair-ratio",
        str(args.junction_mixed_max_opposite_face_pair_ratio),
        "--junction-mixed-scored-min-neighbor-count",
        str(args.junction_mixed_scored_min_neighbor_count),
        "--junction-mixed-scored-min-cluster-count",
        str(args.junction_mixed_scored_min_cluster_count),
        "--junction-mixed-scored-min-junction-score",
        str(args.junction_mixed_scored_min_junction_score),
        "--junction-mixed-scored-min-orientation-dispersion",
        str(args.junction_mixed_scored_min_orientation_dispersion),
        "--junction-mixed-scored-max-dominant-fraction",
        str(args.junction_mixed_scored_max_dominant_fraction),
        "--junction-mixed-scored-min-occupancy-asymmetry",
        str(args.junction_mixed_scored_min_occupancy_asymmetry),
        "--junction-mixed-scored-min-normal-variation",
        str(args.junction_mixed_scored_min_normal_variation),
        "--junction-mixed-scored-threshold",
        str(args.junction_mixed_scored_threshold),
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
    topk_counts, topk_total = parse_topk_counts(run_output_dir / "interesting_voxels.csv")
    label_counts = Counter(summary.get("label_counts", {}))
    base_label_counts = Counter(summary.get("base_label_counts", {}))

    return {
        "mode": ranking_mode,
        "enable_relabel": enable_relabel,
        "relabel_mode": relabel_mode,
        "output_dir": str(run_output_dir),
        "analyzed_voxels": int(summary["analyzed_voxels"]),
        "interesting_top_k": topk_total,
        "junction_like_mixed_count": int(summary.get("junction_like_mixed_count", 0)),
        "combined_corner_or_junction_count": int(summary.get("combined_corner_or_junction_count", 0)),
        "planar_reduction_count": int(summary.get("planar_reduction_count", 0)),
        "interesting_junction_like_mixed_count": int(summary.get("interesting_junction_like_mixed_count", 0)),
        "interesting_combined_corner_or_junction_count": int(summary.get("interesting_combined_corner_or_junction_count", 0)),
        "interesting_planar_reduction_count": int(summary.get("interesting_planar_reduction_count", 0)),
        "average_junction_score": float(summary.get("average_junction_score", 0.0)),
        "average_junction_orientation_dispersion": float(summary.get("average_junction_orientation_dispersion", 0.0)),
        "average_junction_mixed_relabel_score": float(summary.get("average_junction_mixed_relabel_score", 0.0)),
        "label_counts": dict(label_counts),
        "base_label_counts": dict(base_label_counts),
        "topk_counts": dict(topk_counts),
        "options": {
            "junction_mixed_relabel_mode": relabel_mode,
            "junction_mixed_min_neighbor_count": args.junction_mixed_min_neighbor_count,
            "junction_mixed_min_cluster_count": args.junction_mixed_min_cluster_count,
            "junction_mixed_min_score": args.junction_mixed_min_score,
            "junction_mixed_min_orientation_dispersion": args.junction_mixed_min_orientation_dispersion,
            "junction_mixed_max_dominant_fraction": args.junction_mixed_max_dominant_fraction,
            "junction_mixed_min_occupancy_asymmetry": args.junction_mixed_min_occupancy_asymmetry,
            "junction_mixed_min_normal_variation": args.junction_mixed_min_normal_variation,
            "junction_mixed_max_opposite_face_pair_ratio": args.junction_mixed_max_opposite_face_pair_ratio,
            "junction_mixed_scored_min_neighbor_count": args.junction_mixed_scored_min_neighbor_count,
            "junction_mixed_scored_min_cluster_count": args.junction_mixed_scored_min_cluster_count,
            "junction_mixed_scored_min_junction_score": args.junction_mixed_scored_min_junction_score,
            "junction_mixed_scored_min_orientation_dispersion": args.junction_mixed_scored_min_orientation_dispersion,
            "junction_mixed_scored_max_dominant_fraction": args.junction_mixed_scored_max_dominant_fraction,
            "junction_mixed_scored_min_occupancy_asymmetry": args.junction_mixed_scored_min_occupancy_asymmetry,
            "junction_mixed_scored_min_normal_variation": args.junction_mixed_scored_min_normal_variation,
            "junction_mixed_scored_threshold": args.junction_mixed_scored_threshold,
        },
    }


def write_comparison_csv(output_path: Path, rows: list[dict[str, Any]]) -> None:
    baseline = rows[0]
    labels = sorted(
        {
            label
            for row in rows
            for bucket in (row["topk_counts"], row["label_counts"])
            for label in bucket
        }
    )
    fieldnames = [
        "mode",
        "enable_relabel",
        "relabel_mode",
        "output_dir",
        "analyzed_voxels",
        "interesting_top_k",
        "junction_like_mixed_count",
        "combined_corner_or_junction_count",
        "planar_reduction_count",
        "interesting_junction_like_mixed_count",
        "interesting_combined_corner_or_junction_count",
        "interesting_planar_reduction_count",
        "relabeled_planar_voxel_count",
        "topk_relabeled_planar_voxel_count",
        "planar_delta_vs_baseline_topk",
        "combined_delta_vs_baseline_topk",
    ]
    for prefix in ("label", "topk"):
        for label in labels:
            fieldnames.append(f"{prefix}_{label}_count")
            fieldnames.append(f"{prefix}_{label}_fraction")

    baseline_topk_planar = baseline["topk_counts"].get("planar", 0)
    baseline_combined = baseline["topk_counts"].get("corner_like", 0) + baseline["topk_counts"].get("junction_like_mixed", 0)
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            flat = {key: row[key] for key in fieldnames if key in row}
            flat["relabeled_planar_voxel_count"] = row["junction_like_mixed_count"]
            flat["topk_relabeled_planar_voxel_count"] = row["interesting_junction_like_mixed_count"]
            flat["planar_delta_vs_baseline_topk"] = row["topk_counts"].get("planar", 0) - baseline_topk_planar
            flat["combined_delta_vs_baseline_topk"] = (
                row["topk_counts"].get("corner_like", 0) + row["topk_counts"].get("junction_like_mixed", 0) - baseline_combined
            )
            for prefix, counts_key in (("label", "label_counts"), ("topk", "topk_counts")):
                counts = row[counts_key]
                total = max(sum(counts.values()), 1)
                for label in labels:
                    count = counts.get(label, 0)
                    flat[f"{prefix}_{label}_count"] = count
                    flat[f"{prefix}_{label}_fraction"] = count / total
            writer.writerow(flat)


def write_summary_md(output_path: Path, rows: list[dict[str, Any]]) -> None:
    baseline, hard_threshold, scored = rows
    baseline_topk = baseline["topk_counts"]
    baseline_combined = baseline_topk.get("corner_like", 0) + baseline_topk.get("junction_like_mixed", 0)
    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# Junction Mixed Scored Reclassification Summary\n\n")
        handle.write("## Top-k Comparison\n\n")
        for row in rows:
            topk_counts = row["topk_counts"]
            combined = topk_counts.get("corner_like", 0) + topk_counts.get("junction_like_mixed", 0)
            handle.write(
                f"- `{row['mode']}` (`{row['relabel_mode']}`): "
                f"corner_like={topk_counts.get('corner_like', 0)}, "
                f"junction_like_mixed={topk_counts.get('junction_like_mixed', 0)}, "
                f"combined={combined}, "
                f"planar={topk_counts.get('planar', 0)}, "
                f"topk_relabeled={row['interesting_junction_like_mixed_count']}\n"
            )
        handle.write("\n")
        for row in (hard_threshold, scored):
            topk_counts = row["topk_counts"]
            combined = topk_counts.get("corner_like", 0) + topk_counts.get("junction_like_mixed", 0)
            handle.write(
                f"- `{row['mode']}` combined delta vs baseline: {combined - baseline_combined:+d}, "
                f"planar delta vs baseline: {topk_counts.get('planar', 0) - baseline_topk.get('planar', 0):+d}\n"
            )

        handle.write("\n## Global Relabel Counts\n\n")
        for row in rows:
            handle.write(
                f"- `{row['mode']}`: "
                f"junction_like_mixed={row['junction_like_mixed_count']}, "
                f"combined={row['combined_corner_or_junction_count']}, "
                f"planar_reduction={row['planar_reduction_count']}, "
                f"average_relabel_score={row['average_junction_mixed_relabel_score']:.4f}\n"
            )

        handle.write("\n## Parameters\n\n")
        for key, value in scored["options"].items():
            handle.write(f"- `{key}` = `{value}`\n")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run hard-threshold and scored planar -> junction_like_mixed reclassification experiments."
    )
    parser.add_argument("--input", required=True)
    parser.add_argument("--output-root", default="results/context_refinement_scored")
    parser.add_argument("--voxel-size", type=float, default=1.0)
    parser.add_argument("--min-points-per-voxel", type=int, default=15)
    parser.add_argument("--shape-exponent", type=float, default=1.0)
    parser.add_argument("--axis-scale-quantile", type=float, default=0.95)
    parser.add_argument("--top-k", type=int, default=200)
    parser.add_argument("--junction-mixed-min-neighbor-count", type=int, default=10)
    parser.add_argument("--junction-mixed-min-cluster-count", type=int, default=3)
    parser.add_argument("--junction-mixed-min-score", type=float, default=0.72)
    parser.add_argument("--junction-mixed-min-orientation-dispersion", type=float, default=0.48)
    parser.add_argument("--junction-mixed-max-dominant-fraction", type=float, default=0.38)
    parser.add_argument("--junction-mixed-min-occupancy-asymmetry", type=float, default=0.30)
    parser.add_argument("--junction-mixed-min-normal-variation", type=float, default=0.20)
    parser.add_argument("--junction-mixed-max-opposite-face-pair-ratio", type=float, default=0.67)
    parser.add_argument("--junction-mixed-scored-min-neighbor-count", type=int, default=8)
    parser.add_argument("--junction-mixed-scored-min-cluster-count", type=int, default=2)
    parser.add_argument("--junction-mixed-scored-min-junction-score", type=float, default=0.30)
    parser.add_argument("--junction-mixed-scored-min-orientation-dispersion", type=float, default=0.35)
    parser.add_argument("--junction-mixed-scored-max-dominant-fraction", type=float, default=0.62)
    parser.add_argument("--junction-mixed-scored-min-occupancy-asymmetry", type=float, default=0.18)
    parser.add_argument("--junction-mixed-scored-min-normal-variation", type=float, default=0.10)
    parser.add_argument("--junction-mixed-scored-threshold", type=float, default=0.66)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    input_path = (repo_root / args.input).resolve() if not Path(args.input).is_absolute() else Path(args.input)
    output_root = (repo_root / args.output_root).resolve()
    runs_root = output_root / "junction_mixed_scored_runs"
    runs_root.mkdir(parents=True, exist_ok=True)

    analyzer = repo_root / "build" / "ges_voxel_mapping_cpp" / "voxel_morphology_analyzer"
    if not analyzer.exists():
        subprocess.run([str(repo_root / "scripts" / "build_offline.sh")], check=True)

    rows = [
        run_mode(
            analyzer,
            input_path,
            runs_root / "context_hybrid_baseline",
            args,
            ranking_mode="context_hybrid",
            enable_relabel=False,
            relabel_mode="hard_threshold",
        ),
        run_mode(
            analyzer,
            input_path,
            runs_root / "junction_mixed_priority_hard_threshold",
            args,
            ranking_mode="junction_mixed_priority",
            enable_relabel=True,
            relabel_mode="hard_threshold",
        ),
        run_mode(
            analyzer,
            input_path,
            runs_root / "junction_mixed_scored",
            args,
            ranking_mode="junction_mixed_scored",
            enable_relabel=True,
            relabel_mode="scored",
        ),
    ]

    write_comparison_csv(output_root / "junction_mixed_scored_comparison.csv", rows)
    write_summary_md(output_root / "junction_mixed_scored_summary.md", rows)
    print(f"Wrote {output_root / 'junction_mixed_scored_comparison.csv'}")
    print(f"Wrote {output_root / 'junction_mixed_scored_summary.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
