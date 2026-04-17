#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Any

import numpy as np

from run_junction_mixed_scored_calibration import run_mode
from voxel_morphology_fallback import load_point_cloud_file, write_xyz_pcd


def load_voxel_rows(csv_path: Path) -> dict[str, dict[str, str]]:
    rows: dict[str, dict[str, str]] = {}
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows[f"{row['vx']}_{row['vy']}_{row['vz']}"] = row
    return rows


def load_rank_lookup(csv_path: Path) -> dict[str, int]:
    ranks: dict[str, int] = {}
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            ranks[f"{row['vx']}_{row['vy']}_{row['vz']}"] = int(row["rank"])
    return ranks


def load_run_data(run_result: dict[str, Any]) -> dict[str, Any]:
    output_dir = Path(run_result["output_dir"])
    voxel_rows = load_voxel_rows(output_dir / "voxel_metrics.csv")
    ranks = load_rank_lookup(output_dir / "interesting_voxels.csv")
    relabeled_ids = {
        voxel_id
        for voxel_id, row in voxel_rows.items()
        if row["base_label"] == "planar" and row["label"] == "junction_like_mixed"
    }
    return {
        "result": run_result,
        "voxel_rows": voxel_rows,
        "ranks": ranks,
        "relabeled_ids": relabeled_ids,
    }


def build_group_rows(
    baseline_data: dict[str, Any],
    hard_data: dict[str, Any],
    scored_data: dict[str, Any],
) -> list[dict[str, Any]]:
    hard_ids = hard_data["relabeled_ids"]
    scored_ids = scored_data["relabeled_ids"]
    shared_ids = hard_ids & scored_ids
    scored_only_ids = scored_ids - hard_ids
    hard_only_ids = hard_ids - scored_ids

    group_order = {
        "shared_relabeled": 0,
        "scored_only_relabeled": 1,
        "hard_only_relabeled": 2,
    }
    groups = {
        "shared_relabeled": shared_ids,
        "scored_only_relabeled": scored_only_ids,
        "hard_only_relabeled": hard_only_ids,
    }

    rows: list[dict[str, Any]] = []
    for group, voxel_ids in groups.items():
        for voxel_id in sorted(voxel_ids):
            source_row = scored_data["voxel_rows"].get(voxel_id) or hard_data["voxel_rows"].get(voxel_id)
            if source_row is None:
                continue
            hard_rank = hard_data["ranks"].get(voxel_id)
            scored_rank = scored_data["ranks"].get(voxel_id)
            baseline_rank = baseline_data["ranks"].get(voxel_id)
            rows.append(
                {
                    "voxel_id": voxel_id,
                    "vx": source_row["vx"],
                    "vy": source_row["vy"],
                    "vz": source_row["vz"],
                    "group": group,
                    "base_label": source_row["base_label"],
                    "final_label": source_row["label"],
                    "relabel_mode": {
                        "shared_relabeled": "hard_threshold+scored",
                        "scored_only_relabeled": "scored",
                        "hard_only_relabeled": "hard_threshold",
                    }[group],
                    "relabel_score": source_row["junction_mixed_relabel_score"],
                    "junction_score": source_row["junction_score"],
                    "junction_neighbor_count": source_row["junction_neighbor_count"],
                    "junction_cluster_count": source_row["junction_cluster_count"],
                    "junction_orientation_dispersion": source_row["junction_orientation_dispersion"],
                    "junction_dominant_fraction": source_row["junction_dominant_fraction"],
                    "occupancy_asymmetry": source_row["occupancy_asymmetry"],
                    "normal_variation": source_row["normal_variation"],
                    "opposite_face_pair_ratio": source_row["opposite_face_pair_ratio"],
                    "planar_context_penalty": source_row["planar_context_penalty"],
                    "corner_context_bonus": source_row["corner_context_bonus"],
                    "baseline_in_top_k": "true" if baseline_rank is not None else "false",
                    "baseline_rank": "" if baseline_rank is None else baseline_rank,
                    "hard_in_top_k": "true" if hard_rank is not None else "false",
                    "hard_rank": "" if hard_rank is None else hard_rank,
                    "scored_in_top_k": "true" if scored_rank is not None else "false",
                    "scored_rank": "" if scored_rank is None else scored_rank,
                }
            )

    rows.sort(
        key=lambda row: (
            group_order[row["group"]],
            10_000 if row["scored_rank"] == "" else int(row["scored_rank"]),
            10_000 if row["hard_rank"] == "" else int(row["hard_rank"]),
            row["voxel_id"],
        )
    )
    return rows


def compute_voxel_keys(points: np.ndarray, voxel_size: float) -> np.ndarray:
    return np.floor(points[:, :3] / voxel_size).astype(np.int32)


def export_group_artifacts(
    output_root: Path,
    group_rows: list[dict[str, Any]],
    input_points: np.ndarray,
    point_keys: np.ndarray,
    neighborhood_radius: int,
) -> None:
    for group in ("shared_relabeled", "scored_only_relabeled", "hard_only_relabeled"):
        group_dir = output_root / "groups" / group
        voxels_dir = group_dir / "voxels"
        group_dir.mkdir(parents=True, exist_ok=True)
        manifest_rows = []
        combined_voxel_mask = np.zeros((input_points.shape[0],), dtype=bool)
        combined_neighborhood_mask = np.zeros((input_points.shape[0],), dtype=bool)

        current_rows = [row for row in group_rows if row["group"] == group]
        for row in current_rows:
            key = np.array([int(row["vx"]), int(row["vy"]), int(row["vz"])], dtype=np.int32)
            voxel_mask = np.all(point_keys == key[None, :], axis=1)
            neighborhood_mask = np.max(np.abs(point_keys - key[None, :]), axis=1) <= neighborhood_radius

            combined_voxel_mask |= voxel_mask
            combined_neighborhood_mask |= neighborhood_mask

            voxel_dir = voxels_dir / row["voxel_id"]
            voxel_dir.mkdir(parents=True, exist_ok=True)
            voxel_path = voxel_dir / "voxel.pcd"
            neighborhood_path = voxel_dir / f"neighborhood_radius{neighborhood_radius}.pcd"

            write_xyz_pcd(voxel_path, input_points[voxel_mask])
            write_xyz_pcd(neighborhood_path, input_points[neighborhood_mask])

            row["voxel_pcd_path"] = str(voxel_path.relative_to(output_root))
            row["neighborhood_pcd_path"] = str(neighborhood_path.relative_to(output_root))

            manifest_rows.append(
                {
                    "voxel_id": row["voxel_id"],
                    "group": row["group"],
                    "baseline_rank": row["baseline_rank"],
                    "hard_rank": row["hard_rank"],
                    "scored_rank": row["scored_rank"],
                    "voxel_pcd_path": row["voxel_pcd_path"],
                    "neighborhood_pcd_path": row["neighborhood_pcd_path"],
                }
            )

        combined_voxel_path = group_dir / "combined_voxels.pcd"
        combined_neighborhood_path = group_dir / f"combined_neighborhood_radius{neighborhood_radius}.pcd"
        write_xyz_pcd(combined_voxel_path, input_points[combined_voxel_mask])
        write_xyz_pcd(combined_neighborhood_path, input_points[combined_neighborhood_mask])

        with (group_dir / "manifest.csv").open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "voxel_id",
                    "group",
                    "baseline_rank",
                    "hard_rank",
                    "scored_rank",
                    "voxel_pcd_path",
                    "neighborhood_pcd_path",
                ],
            )
            writer.writeheader()
            for manifest_row in manifest_rows:
                writer.writerow(manifest_row)


def write_review_csv(output_path: Path, group_rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "voxel_id",
        "vx",
        "vy",
        "vz",
        "group",
        "base_label",
        "final_label",
        "relabel_mode",
        "relabel_score",
        "junction_score",
        "junction_neighbor_count",
        "junction_cluster_count",
        "junction_orientation_dispersion",
        "junction_dominant_fraction",
        "occupancy_asymmetry",
        "normal_variation",
        "opposite_face_pair_ratio",
        "planar_context_penalty",
        "corner_context_bonus",
        "baseline_in_top_k",
        "baseline_rank",
        "hard_in_top_k",
        "hard_rank",
        "scored_in_top_k",
        "scored_rank",
        "voxel_pcd_path",
        "neighborhood_pcd_path",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in group_rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def summarize_group(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {
            "count": 0,
            "baseline_topk_count": 0,
            "hard_topk_count": 0,
            "scored_topk_count": 0,
            "mean_relabel_score": 0.0,
            "mean_junction_score": 0.0,
            "mean_cluster_count": 0.0,
            "mean_dispersion": 0.0,
            "mean_dominant_fraction": 0.0,
            "mean_occupancy_asymmetry": 0.0,
            "mean_normal_variation": 0.0,
        }

    return {
        "count": len(rows),
        "baseline_topk_count": sum(row["baseline_in_top_k"] == "true" for row in rows),
        "hard_topk_count": sum(row["hard_in_top_k"] == "true" for row in rows),
        "scored_topk_count": sum(row["scored_in_top_k"] == "true" for row in rows),
        "mean_relabel_score": float(np.mean([float(row["relabel_score"]) for row in rows])),
        "mean_junction_score": float(np.mean([float(row["junction_score"]) for row in rows])),
        "mean_cluster_count": float(np.mean([float(row["junction_cluster_count"]) for row in rows])),
        "mean_dispersion": float(np.mean([float(row["junction_orientation_dispersion"]) for row in rows])),
        "mean_dominant_fraction": float(np.mean([float(row["junction_dominant_fraction"]) for row in rows])),
        "mean_occupancy_asymmetry": float(np.mean([float(row["occupancy_asymmetry"]) for row in rows])),
        "mean_normal_variation": float(np.mean([float(row["normal_variation"]) for row in rows])),
    }


def write_review_md(
    output_path: Path,
    output_root: Path,
    group_rows: list[dict[str, Any]],
    neighborhood_radius: int,
) -> None:
    groups = {
        group: [row for row in group_rows if row["group"] == group]
        for group in ("shared_relabeled", "scored_only_relabeled", "hard_only_relabeled")
    }
    group_stats = {group: summarize_group(rows) for group, rows in groups.items()}
    scored_only_ids = [row["voxel_id"] for row in groups["scored_only_relabeled"]]
    hard_only_ids = [row["voxel_id"] for row in groups["hard_only_relabeled"]]

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# Junction-Like Mixed Review Export Summary\n\n")
        handle.write("## Group Counts\n\n")
        for group in ("shared_relabeled", "scored_only_relabeled", "hard_only_relabeled"):
            stats = group_stats[group]
            handle.write(
                f"- `{group}`: count={stats['count']}, "
                f"baseline_topk={stats['baseline_topk_count']}, "
                f"hard_topk={stats['hard_topk_count']}, "
                f"scored_topk={stats['scored_topk_count']}\n"
            )

        handle.write("\n## Group Feature Means\n\n")
        for group in ("shared_relabeled", "scored_only_relabeled", "hard_only_relabeled"):
            stats = group_stats[group]
            handle.write(
                f"- `{group}`: relabel_score={stats['mean_relabel_score']:.4f}, "
                f"junction_score={stats['mean_junction_score']:.4f}, "
                f"cluster_count={stats['mean_cluster_count']:.2f}, "
                f"dispersion={stats['mean_dispersion']:.4f}, "
                f"dominant_fraction={stats['mean_dominant_fraction']:.4f}, "
                f"occupancy_asymmetry={stats['mean_occupancy_asymmetry']:.4f}, "
                f"normal_variation={stats['mean_normal_variation']:.4f}\n"
            )

        handle.write("\n## Exact Voxel IDs\n\n")
        handle.write(f"- `scored_only_relabeled`: {', '.join(scored_only_ids) if scored_only_ids else '(none)'}\n")
        handle.write(f"- `hard_only_relabeled`: {', '.join(hard_only_ids) if hard_only_ids else '(none)'}\n")

        handle.write("\n## Files To Open First\n\n")
        handle.write(
            f"- scored-only combined neighborhood: "
            f"`groups/scored_only_relabeled/combined_neighborhood_radius{neighborhood_radius}.pcd`\n"
        )
        handle.write(
            f"- hard-only combined neighborhood: "
            f"`groups/hard_only_relabeled/combined_neighborhood_radius{neighborhood_radius}.pcd`\n"
        )
        handle.write(
            f"- full review table: `{Path(output_path).with_suffix('.csv').relative_to(output_root)}`\n"
        )
        scored_topk_rows = [row for row in groups["scored_only_relabeled"] if row["scored_in_top_k"] == "true"]
        if scored_topk_rows:
            handle.write("- scored-only top-k voxel neighborhoods to inspect first:\n")
            for row in scored_topk_rows:
                handle.write(
                    f"  - voxel={row['voxel_id']}, scored_rank={row['scored_rank']}, "
                    f"`{row['neighborhood_pcd_path']}`\n"
                )
        hard_rows = groups["hard_only_relabeled"]
        if hard_rows:
            handle.write("- hard-only voxel neighborhoods:\n")
            for row in hard_rows:
                rank = row["hard_rank"] if row["hard_rank"] != "" else "out_of_top_k"
                handle.write(
                    f"  - voxel={row['voxel_id']}, hard_rank={rank}, "
                    f"`{row['neighborhood_pcd_path']}`\n"
                )

        handle.write("\n## Manifests\n\n")
        for group in ("shared_relabeled", "scored_only_relabeled", "hard_only_relabeled"):
            handle.write(f"- `{group}` manifest: `groups/{group}/manifest.csv`\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Export review artifacts for hard/scored junction_like_mixed voxels.")
    parser.add_argument("--input", required=True)
    parser.add_argument("--output-root", default="results/context_refinement_scored_review")
    parser.add_argument("--voxel-size", type=float, default=1.0)
    parser.add_argument("--min-points-per-voxel", type=int, default=15)
    parser.add_argument("--shape-exponent", type=float, default=1.0)
    parser.add_argument("--axis-scale-quantile", type=float, default=0.95)
    parser.add_argument("--review-threshold", type=float, default=0.66)
    parser.add_argument("--top-k", type=int, default=200)
    parser.add_argument("--neighborhood-radius", type=int, default=2)
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
    runs_root = output_root / "runs"
    runs_root.mkdir(parents=True, exist_ok=True)

    analyzer = repo_root / "build" / "ges_voxel_mapping_cpp" / "voxel_morphology_analyzer"
    if not analyzer.exists():
        raise FileNotFoundError(f"Analyzer not found: {analyzer}")

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
    scored_result = run_mode(
        analyzer,
        input_path,
        runs_root / "junction_mixed_scored_t0_66",
        args,
        ranking_mode="junction_mixed_scored",
        enable_relabel=True,
        relabel_mode="scored",
        score_threshold=args.review_threshold,
    )

    baseline_data = load_run_data(baseline)
    hard_data = load_run_data(hard_result)
    scored_data = load_run_data(scored_result)
    group_rows = build_group_rows(baseline_data, hard_data, scored_data)

    input_points, _ = load_point_cloud_file(str(input_path))
    point_keys = compute_voxel_keys(input_points, args.voxel_size)
    export_group_artifacts(output_root, group_rows, input_points, point_keys, args.neighborhood_radius)

    review_csv = output_root / "relabeled_review_summary.csv"
    review_md = output_root / "relabeled_review_summary.md"
    write_review_csv(review_csv, group_rows)
    write_review_md(review_md, output_root, group_rows, args.neighborhood_radius)

    print(f"Wrote {review_csv}")
    print(f"Wrote {review_md}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
