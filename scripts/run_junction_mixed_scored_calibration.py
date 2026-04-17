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


def parse_thresholds(text: str) -> list[float]:
    values = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        values.append(float(part))
    if not values:
        raise ValueError("score threshold list is empty")
    return values


def threshold_tag(value: float) -> str:
    return f"{value:.2f}".replace(".", "_")


def load_rank_lookup(csv_path: Path) -> dict[tuple[str, str, str], int]:
    rank_lookup: dict[tuple[str, str, str], int] = {}
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rank_lookup[(row["vx"], row["vy"], row["vz"])] = int(row["rank"])
    return rank_lookup


def build_review_rows(
    run_result: dict[str, Any],
    top_k: int,
) -> list[dict[str, Any]]:
    voxel_metrics_path = Path(run_result["output_dir"]) / "voxel_metrics.csv"
    rank_lookup = load_rank_lookup(Path(run_result["output_dir"]) / "interesting_voxels.csv")
    review_rows: list[dict[str, Any]] = []

    with voxel_metrics_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row["base_label"] != "planar" or row["label"] != "junction_like_mixed":
                continue
            key = (row["vx"], row["vy"], row["vz"])
            ranking_position = rank_lookup.get(key)
            review_rows.append(
                {
                    "voxel_id": f"{row['vx']}_{row['vy']}_{row['vz']}",
                    "vx": row["vx"],
                    "vy": row["vy"],
                    "vz": row["vz"],
                    "base_label": row["base_label"],
                    "label": row["label"],
                    "ranking_mode": run_result["mode"],
                    "relabel_mode": run_result["relabel_mode"],
                    "score_threshold": run_result["score_threshold_text"],
                    "relabel_score": row["junction_mixed_relabel_score"],
                    "junction_score": row["junction_score"],
                    "junction_neighbor_count": row["junction_neighbor_count"],
                    "junction_cluster_count": row["junction_cluster_count"],
                    "junction_orientation_dispersion": row["junction_orientation_dispersion"],
                    "junction_dominant_fraction": row["junction_dominant_fraction"],
                    "occupancy_asymmetry": row["occupancy_asymmetry"],
                    "normal_variation": row["normal_variation"],
                    "opposite_face_pair_ratio": row["opposite_face_pair_ratio"],
                    "ranking_position": "" if ranking_position is None else ranking_position,
                    "in_top_k": "true" if ranking_position is not None and ranking_position < top_k else "false",
                    "output_dir": run_result["output_dir"],
                }
            )

    review_rows.sort(
        key=lambda row: (
            row["relabel_mode"] != "hard_threshold",
            1 if row["ranking_position"] == "" else 0,
            10_000 if row["ranking_position"] == "" else int(row["ranking_position"]),
            -float(row["relabel_score"]),
        )
    )
    return review_rows


def run_mode(
    analyzer: Path,
    input_path: Path,
    run_output_dir: Path,
    args: argparse.Namespace,
    ranking_mode: str,
    enable_relabel: bool,
    relabel_mode: str,
    score_threshold: float | None,
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
        str(args.junction_mixed_scored_threshold if score_threshold is None else score_threshold),
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

    result = {
        "mode": ranking_mode,
        "relabel_mode": relabel_mode,
        "enable_relabel": enable_relabel,
        "score_threshold": score_threshold,
        "score_threshold_text": "" if score_threshold is None else f"{score_threshold:.2f}",
        "output_dir": str(run_output_dir),
        "analyzed_voxels": int(summary["analyzed_voxels"]),
        "interesting_top_k": topk_total,
        "label_counts": dict(label_counts),
        "base_label_counts": dict(base_label_counts),
        "topk_counts": dict(topk_counts),
        "junction_like_mixed_count": int(summary.get("junction_like_mixed_count", 0)),
        "combined_corner_or_junction_count": int(summary.get("combined_corner_or_junction_count", 0)),
        "planar_reduction_count": int(summary.get("planar_reduction_count", 0)),
        "interesting_junction_like_mixed_count": int(summary.get("interesting_junction_like_mixed_count", 0)),
        "interesting_combined_corner_or_junction_count": int(summary.get("interesting_combined_corner_or_junction_count", 0)),
        "interesting_planar_reduction_count": int(summary.get("interesting_planar_reduction_count", 0)),
        "average_junction_mixed_relabel_score": float(summary.get("average_junction_mixed_relabel_score", 0.0)),
        "average_junction_mixed_relabel_score_all_voxels": float(
            summary.get("average_junction_mixed_relabel_score_all_voxels", summary.get("average_junction_mixed_relabel_score", 0.0))
        ),
        "average_planar_junction_mixed_relabel_score": float(
            summary.get("average_planar_junction_mixed_relabel_score", 0.0)
        ),
        "average_relabeled_junction_mixed_relabel_score": float(
            summary.get("average_relabeled_junction_mixed_relabel_score", 0.0)
        ),
        "average_topk_relabeled_junction_mixed_relabel_score": float(
            summary.get("average_topk_relabeled_junction_mixed_relabel_score", 0.0)
        ),
    }
    result["review_rows"] = build_review_rows(result, args.top_k)
    return result


def write_review_csv(output_path: Path, review_rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "voxel_id",
        "vx",
        "vy",
        "vz",
        "base_label",
        "label",
        "ranking_mode",
        "relabel_mode",
        "score_threshold",
        "relabel_score",
        "junction_score",
        "junction_neighbor_count",
        "junction_cluster_count",
        "junction_orientation_dispersion",
        "junction_dominant_fraction",
        "occupancy_asymmetry",
        "normal_variation",
        "opposite_face_pair_ratio",
        "ranking_position",
        "in_top_k",
        "output_dir",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in review_rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def write_review_md(
    output_path: Path,
    hard_result: dict[str, Any],
    scored_result: dict[str, Any],
) -> None:
    hard_ids = {row["voxel_id"] for row in hard_result["review_rows"]}
    scored_ids = {row["voxel_id"] for row in scored_result["review_rows"]}
    shared_ids = hard_ids & scored_ids
    scored_only = [row for row in scored_result["review_rows"] if row["voxel_id"] not in hard_ids]
    hard_only = [row for row in hard_result["review_rows"] if row["voxel_id"] not in scored_ids]

    def topk_preview(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [row for row in rows if row["ranking_position"] != ""][:5]

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# Relabeled Junction-Like Mixed Review\n\n")
        handle.write("## Mode Summary\n\n")
        for result in (hard_result, scored_result):
            handle.write(
                f"- `{result['mode']}` (`{result['relabel_mode']}`, threshold={result['score_threshold_text'] or 'n/a'}): "
                f"relabeled={len(result['review_rows'])}, "
                f"top-k relabeled={result['interesting_junction_like_mixed_count']}, "
                f"avg relabeled score={result['average_relabeled_junction_mixed_relabel_score']:.4f}\n"
            )

        handle.write("\n## Overlap\n\n")
        handle.write(f"- shared relabeled voxels: {len(shared_ids)}\n")
        handle.write(f"- scored-only relabeled voxels: {len(scored_only)}\n")
        handle.write(f"- hard-threshold-only relabeled voxels: {len(hard_only)}\n")

        handle.write("\n## Top-k Preview\n\n")
        for title, rows in (
            ("hard-threshold", topk_preview(hard_result["review_rows"])),
            ("scored", topk_preview(scored_result["review_rows"])),
        ):
            handle.write(f"- `{title}`:\n")
            if not rows:
                handle.write("  none\n")
                continue
            for row in rows:
                handle.write(
                    f"  rank={row['ranking_position']}, voxel={row['voxel_id']}, "
                    f"relabel_score={float(row['relabel_score']):.4f}, "
                    f"junction_score={float(row['junction_score']):.4f}, "
                    f"clusters={row['junction_cluster_count']}, "
                    f"dispersion={float(row['junction_orientation_dispersion']):.4f}\n"
                )

        if scored_only:
            handle.write("\n## Scored-Only Voxels\n\n")
            for row in scored_only[:8]:
                handle.write(
                    f"- voxel={row['voxel_id']}, rank={row['ranking_position'] or 'out_of_top_k'}, "
                    f"relabel_score={float(row['relabel_score']):.4f}, "
                    f"junction_score={float(row['junction_score']):.4f}, "
                    f"dominant_fraction={float(row['junction_dominant_fraction']):.4f}\n"
                )


def write_calibration_csv(output_path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "run_label",
        "ranking_mode",
        "relabel_mode",
        "score_threshold",
        "output_dir",
        "analyzed_voxels",
        "topk_corner_like_count",
        "topk_junction_like_mixed_count",
        "topk_combined_corner_or_junction_count",
        "topk_planar_count",
        "relabeled_planar_voxel_count",
        "topk_relabeled_planar_voxel_count",
        "combined_delta_vs_baseline_topk",
        "planar_delta_vs_baseline_topk",
        "average_junction_mixed_relabel_score_all_voxels",
        "average_planar_junction_mixed_relabel_score",
        "average_relabeled_junction_mixed_relabel_score",
        "average_topk_relabeled_junction_mixed_relabel_score",
    ]
    baseline = rows[0]
    baseline_combined = baseline["topk_counts"].get("corner_like", 0) + baseline["topk_counts"].get("junction_like_mixed", 0)
    baseline_planar = baseline["topk_counts"].get("planar", 0)

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            topk_corner = row["topk_counts"].get("corner_like", 0)
            topk_junction = row["topk_counts"].get("junction_like_mixed", 0)
            topk_combined = topk_corner + topk_junction
            writer.writerow(
                {
                    "run_label": row["mode"] if row["score_threshold_text"] == "" else f"{row['mode']}_t{row['score_threshold_text']}",
                    "ranking_mode": row["mode"],
                    "relabel_mode": row["relabel_mode"],
                    "score_threshold": row["score_threshold_text"],
                    "output_dir": row["output_dir"],
                    "analyzed_voxels": row["analyzed_voxels"],
                    "topk_corner_like_count": topk_corner,
                    "topk_junction_like_mixed_count": topk_junction,
                    "topk_combined_corner_or_junction_count": topk_combined,
                    "topk_planar_count": row["topk_counts"].get("planar", 0),
                    "relabeled_planar_voxel_count": row["junction_like_mixed_count"],
                    "topk_relabeled_planar_voxel_count": row["interesting_junction_like_mixed_count"],
                    "combined_delta_vs_baseline_topk": topk_combined - baseline_combined,
                    "planar_delta_vs_baseline_topk": row["topk_counts"].get("planar", 0) - baseline_planar,
                    "average_junction_mixed_relabel_score_all_voxels": row["average_junction_mixed_relabel_score_all_voxels"],
                    "average_planar_junction_mixed_relabel_score": row["average_planar_junction_mixed_relabel_score"],
                    "average_relabeled_junction_mixed_relabel_score": row["average_relabeled_junction_mixed_relabel_score"],
                    "average_topk_relabeled_junction_mixed_relabel_score": row["average_topk_relabeled_junction_mixed_relabel_score"],
                }
            )


def write_calibration_md(output_path: Path, rows: list[dict[str, Any]], review_threshold: float) -> None:
    baseline = rows[0]
    baseline_topk_combined = baseline["topk_counts"].get("corner_like", 0) + baseline["topk_counts"].get("junction_like_mixed", 0)
    baseline_planar = baseline["topk_counts"].get("planar", 0)
    hard_result = rows[1]
    scored_rows = rows[2:]
    current_scored = next(row for row in scored_rows if abs((row["score_threshold"] or 0.0) - review_threshold) < 1e-9)

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# Junction Mixed Scored Calibration Summary\n\n")
        handle.write("## Baseline and Hard-Threshold\n\n")
        for row in (baseline, hard_result, current_scored):
            topk_combined = row["topk_counts"].get("corner_like", 0) + row["topk_counts"].get("junction_like_mixed", 0)
            handle.write(
                f"- `{row['mode']}` (`{row['relabel_mode']}`, threshold={row['score_threshold_text'] or 'n/a'}): "
                f"combined={topk_combined}, planar={row['topk_counts'].get('planar', 0)}, "
                f"relabeled={row['junction_like_mixed_count']}, top-k relabeled={row['interesting_junction_like_mixed_count']}\n"
            )

        handle.write("\n## Scored Threshold Sweep\n\n")
        for row in scored_rows:
            topk_combined = row["topk_counts"].get("corner_like", 0) + row["topk_counts"].get("junction_like_mixed", 0)
            handle.write(
                f"- threshold `{row['score_threshold_text']}`: "
                f"combined={topk_combined} ({topk_combined - baseline_topk_combined:+d}), "
                f"planar={row['topk_counts'].get('planar', 0)} ({row['topk_counts'].get('planar', 0) - baseline_planar:+d}), "
                f"relabeled={row['junction_like_mixed_count']}, "
                f"top-k relabeled={row['interesting_junction_like_mixed_count']}\n"
            )

        handle.write("\n## Relabel Score Aggregate Check\n\n")
        handle.write(
            "- `average_junction_mixed_relabel_score` and `average_junction_mixed_relabel_score_all_voxels` are expected "
            "to be identical across modes because the relabel score is computed once per analyzed voxel before thresholding.\n"
        )
        handle.write(
            "- Mode-sensitive aggregates are `average_relabeled_junction_mixed_relabel_score` and "
            "`average_topk_relabeled_junction_mixed_relabel_score`.\n"
        )
        handle.write(
            f"- Current scored threshold `{current_scored['score_threshold_text']}`: "
            f"all_voxels={current_scored['average_junction_mixed_relabel_score_all_voxels']:.4f}, "
            f"planar={current_scored['average_planar_junction_mixed_relabel_score']:.4f}, "
            f"relabeled={current_scored['average_relabeled_junction_mixed_relabel_score']:.4f}, "
            f"top-k relabeled={current_scored['average_topk_relabeled_junction_mixed_relabel_score']:.4f}\n"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate and calibrate scored junction_like_mixed relabeling.")
    parser.add_argument("--input", required=True)
    parser.add_argument("--output-root", default="results/context_refinement_scored_calibration")
    parser.add_argument("--voxel-size", type=float, default=1.0)
    parser.add_argument("--min-points-per-voxel", type=int, default=15)
    parser.add_argument("--shape-exponent", type=float, default=1.0)
    parser.add_argument("--axis-scale-quantile", type=float, default=0.95)
    parser.add_argument("--top-k", type=int, default=200)
    parser.add_argument("--score-thresholds", default="0.58,0.62,0.66,0.70,0.74")
    parser.add_argument("--review-threshold", type=float, default=0.66)
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

    thresholds = sorted(set(parse_thresholds(args.score_thresholds) + [args.review_threshold]))
    repo_root = Path(__file__).resolve().parents[1]
    input_path = (repo_root / args.input).resolve() if not Path(args.input).is_absolute() else Path(args.input)
    output_root = (repo_root / args.output_root).resolve()
    runs_root = output_root / "runs"
    runs_root.mkdir(parents=True, exist_ok=True)

    analyzer = repo_root / "build" / "ges_voxel_mapping_cpp" / "voxel_morphology_analyzer"
    if not analyzer.exists():
        subprocess.run([str(repo_root / "scripts" / "build_offline.sh")], check=True)

    baseline = run_mode(
        analyzer,
        input_path,
        runs_root / "context_hybrid_baseline",
        args,
        ranking_mode="context_hybrid",
        enable_relabel=False,
        relabel_mode="hard_threshold",
        score_threshold=None,
    )
    hard_result = run_mode(
        analyzer,
        input_path,
        runs_root / "junction_mixed_priority_hard_threshold",
        args,
        ranking_mode="junction_mixed_priority",
        enable_relabel=True,
        relabel_mode="hard_threshold",
        score_threshold=None,
    )

    scored_results = []
    for threshold in thresholds:
        scored_results.append(
            run_mode(
                analyzer,
                input_path,
                runs_root / f"junction_mixed_scored_t{threshold_tag(threshold)}",
                args,
                ranking_mode="junction_mixed_scored",
                enable_relabel=True,
                relabel_mode="scored",
                score_threshold=threshold,
            )
        )

    current_scored = next(result for result in scored_results if abs((result["score_threshold"] or 0.0) - args.review_threshold) < 1e-9)
    review_rows = hard_result["review_rows"] + current_scored["review_rows"]
    comparison_rows = [baseline, hard_result] + scored_results

    write_review_csv(output_root / "relabeled_voxel_review.csv", review_rows)
    write_review_md(output_root / "relabeled_voxel_review.md", hard_result, current_scored)
    write_calibration_csv(output_root / "scored_calibration_comparison.csv", comparison_rows)
    write_calibration_md(output_root / "scored_calibration_summary.md", comparison_rows, args.review_threshold)

    print(f"Wrote {output_root / 'relabeled_voxel_review.csv'}")
    print(f"Wrote {output_root / 'relabeled_voxel_review.md'}")
    print(f"Wrote {output_root / 'scored_calibration_comparison.csv'}")
    print(f"Wrote {output_root / 'scored_calibration_summary.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
