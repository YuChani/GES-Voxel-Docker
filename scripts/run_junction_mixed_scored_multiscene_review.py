#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from pathlib import Path
from typing import Any

from run_junction_mixed_scored_calibration import run_mode
from run_junction_mixed_scored_review import (
    build_group_rows,
    compute_voxel_keys,
    export_group_artifacts,
    load_run_data,
    summarize_group,
    write_review_csv,
    write_review_md,
)
from voxel_morphology_fallback import load_point_cloud_file

GROUP_NAMES = ("shared_relabeled", "scored_only_relabeled", "hard_only_relabeled")
HUMAN_LABEL_OPTIONS = (
    "junction_like_mixed",
    "planar_boundary",
    "ambiguous",
    "noise_or_bad_case",
)


def scene_sort_key(path: Path) -> tuple[int, str]:
    stem = path.stem
    digits = "".join(char for char in stem if char.isdigit())
    return (int(digits) if digits else 0, path.name)


def parse_scene_names(text: str) -> list[str]:
    names = []
    for part in text.split(","):
        name = part.strip()
        if name:
            names.append(name if name.endswith(".pcd") else f"{name}.pcd")
    return names


def read_pcd_point_count(path: Path) -> int:
    with path.open("rb") as handle:
        for raw_line in handle:
            line = raw_line.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            if line.startswith("POINTS "):
                return int(line.split()[1])
            if line.startswith("DATA "):
                break
    points, _ = load_point_cloud_file(str(path))
    return int(points.shape[0])


def pick_unused_quantile_scene(
    sorted_rows: list[dict[str, Any]],
    quantile: float,
    used_names: set[str],
) -> dict[str, Any]:
    if not sorted_rows:
        raise ValueError("no scenes available for quantile selection")
    center = round((len(sorted_rows) - 1) * quantile)
    for radius in range(len(sorted_rows)):
        for candidate_index in (center - radius, center + radius):
            if candidate_index < 0 or candidate_index >= len(sorted_rows):
                continue
            candidate = sorted_rows[candidate_index]
            if candidate["scene_name"] not in used_names:
                return candidate
    raise ValueError("unable to select a unique representative scene")


def choose_scene_rows(input_root: Path, explicit_scene_names: list[str]) -> list[dict[str, Any]]:
    scene_paths = sorted(input_root.glob("full*.pcd"), key=scene_sort_key)
    if len(scene_paths) < 3:
        raise FileNotFoundError(f"Need at least 3 full*.pcd files under {input_root}")

    rows = [
        {
            "scene_name": path.name,
            "scene_path": str(path),
            "point_count": read_pcd_point_count(path),
        }
        for path in scene_paths
    ]
    by_name = {row["scene_name"]: row for row in rows}

    if explicit_scene_names:
        selected_rows = []
        for scene_name in explicit_scene_names:
            if scene_name not in by_name:
                raise FileNotFoundError(f"Scene not found under {input_root}: {scene_name}")
            row = dict(by_name[scene_name])
            row["selection_reason"] = "user_requested"
            selected_rows.append(row)
        if len(selected_rows) < 3:
            raise ValueError("Please provide at least 3 scene names when using --scene-names")
        return selected_rows

    if "full0.pcd" not in by_name:
        raise FileNotFoundError(f"Expected full0.pcd under {input_root}")

    sorted_by_count = sorted(
        (row for row in rows if row["scene_name"] != "full0.pcd"),
        key=lambda row: (row["point_count"], row["scene_name"]),
    )

    selected = [dict(by_name["full0.pcd"])]
    selected[0]["selection_reason"] = "anchor_full0"
    used_names = {"full0.pcd"}

    for quantile, reason in ((0.25, "point_count_25pct"), (0.75, "point_count_75pct")):
        row = dict(pick_unused_quantile_scene(sorted_by_count, quantile, used_names))
        row["selection_reason"] = reason
        selected.append(row)
        used_names.add(row["scene_name"])

    return selected


def build_annotation_row(row: dict[str, Any]) -> dict[str, Any]:
    annotation_row = dict(row)
    annotation_row["human_label"] = ""
    annotation_row["confidence"] = ""
    annotation_row["notes"] = ""
    return annotation_row


def write_annotation_csv(output_path: Path, rows: list[dict[str, Any]]) -> None:
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
        "human_label",
        "confidence",
        "notes",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def write_annotation_md(output_path: Path, rows: list[dict[str, Any]], title: str) -> None:
    with output_path.open("w", encoding="utf-8") as handle:
        handle.write(f"# {title}\n\n")
        handle.write(
            "Suggested `human_label` values: "
            + ", ".join(f"`{label}`" for label in HUMAN_LABEL_OPTIONS)
            + "\n\n"
        )
        handle.write(
            "| voxel_id | group | relabel_mode | relabel_score | junction_score | "
            "baseline_rank | hard_rank | scored_rank | neighborhood_pcd_path | human_label | confidence | notes |\n"
        )
        handle.write(
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n"
        )
        if not rows:
            handle.write("| (none) |  |  |  |  |  |  |  |  |  |  |  |\n")
            return
        for row in rows:
            handle.write(
                f"| {row['voxel_id']} | {row['group']} | {row['relabel_mode']} | "
                f"{float(row['relabel_score']):.4f} | {float(row['junction_score']):.4f} | "
                f"{row['baseline_rank'] or '-'} | {row['hard_rank'] or '-'} | {row['scored_rank'] or '-'} | "
                f"{row['neighborhood_pcd_path']} |  |  |  |\n"
            )


def write_annotation_templates(scene_output_root: Path, group_rows: list[dict[str, Any]]) -> dict[str, str]:
    output_paths: dict[str, str] = {}
    all_rows = [build_annotation_row(row) for row in group_rows]
    combined_csv = scene_output_root / "annotation_template.csv"
    combined_md = scene_output_root / "annotation_template.md"
    write_annotation_csv(combined_csv, all_rows)
    write_annotation_md(combined_md, all_rows, "Relabeled Voxel Annotation Template")
    output_paths["combined_csv"] = str(combined_csv.relative_to(scene_output_root))
    output_paths["combined_md"] = str(combined_md.relative_to(scene_output_root))

    for group in GROUP_NAMES:
        group_rows_for_annotation = [build_annotation_row(row) for row in group_rows if row["group"] == group]
        group_dir = scene_output_root / "groups" / group
        csv_path = group_dir / "annotation_template.csv"
        md_path = group_dir / "annotation_template.md"
        write_annotation_csv(csv_path, group_rows_for_annotation)
        write_annotation_md(md_path, group_rows_for_annotation, f"{group} Annotation Template")
        output_paths[f"{group}_csv"] = str(csv_path.relative_to(scene_output_root))
        output_paths[f"{group}_md"] = str(md_path.relative_to(scene_output_root))

    return output_paths


def compute_topk_counts(result: dict[str, Any]) -> tuple[int, int]:
    combined = result["topk_counts"].get("corner_like", 0) + result["topk_counts"].get("junction_like_mixed", 0)
    planar = result["topk_counts"].get("planar", 0)
    return combined, planar


def build_group_lookup(group_rows: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    return {group: [row for row in group_rows if row["group"] == group] for group in GROUP_NAMES}


def write_scene_summary_md(
    output_path: Path,
    scene_row: dict[str, Any],
    baseline_result: dict[str, Any],
    hard_result: dict[str, Any],
    scored_result: dict[str, Any],
    group_rows: list[dict[str, Any]],
    annotation_paths: dict[str, str],
    neighborhood_radius: int,
) -> None:
    groups = build_group_lookup(group_rows)
    group_stats = {group: summarize_group(rows) for group, rows in groups.items()}
    baseline_combined, baseline_planar = compute_topk_counts(baseline_result)
    hard_combined, hard_planar = compute_topk_counts(hard_result)
    scored_combined, scored_planar = compute_topk_counts(scored_result)

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write(f"# Scene Review Summary: {scene_row['scene_name']}\n\n")
        handle.write(f"- scene path: `{scene_row['scene_path']}`\n")
        handle.write(f"- point count: {scene_row['point_count']}\n")
        handle.write(f"- selection reason: `{scene_row['selection_reason']}`\n\n")

        handle.write("## Top-k Comparison\n\n")
        handle.write(
            f"- baseline `context_hybrid`: combined={baseline_combined}, planar={baseline_planar}\n"
        )
        handle.write(
            f"- hard-threshold `junction_mixed_priority`: combined={hard_combined}, planar={hard_planar}\n"
        )
        handle.write(
            f"- scored `junction_mixed_scored` (threshold=0.66): combined={scored_combined}, planar={scored_planar}\n"
        )

        handle.write("\n## Group Counts\n\n")
        for group in GROUP_NAMES:
            stats = group_stats[group]
            handle.write(
                f"- `{group}`: count={stats['count']}, baseline_topk={stats['baseline_topk_count']}, "
                f"hard_topk={stats['hard_topk_count']}, scored_topk={stats['scored_topk_count']}\n"
            )

        handle.write("\n## Exact Voxel IDs\n\n")
        for group in GROUP_NAMES:
            ids = [row["voxel_id"] for row in groups[group]]
            handle.write(f"- `{group}`: {', '.join(ids) if ids else '(none)'}\n")

        handle.write("\n## Files To Inspect First\n\n")
        handle.write(
            f"- scored-only combined neighborhood: `groups/scored_only_relabeled/combined_neighborhood_radius{neighborhood_radius}.pcd`\n"
        )
        handle.write(
            f"- scored-only annotation template: `{annotation_paths['scored_only_relabeled_csv']}`\n"
        )
        handle.write(
            f"- hard-only combined neighborhood: `groups/hard_only_relabeled/combined_neighborhood_radius{neighborhood_radius}.pcd`\n"
        )
        handle.write(
            f"- hard-only annotation template: `{annotation_paths['hard_only_relabeled_csv']}`\n"
        )
        handle.write(f"- combined annotation template: `{annotation_paths['combined_csv']}`\n")


def process_scene(
    analyzer: Path,
    scene_row: dict[str, Any],
    output_root: Path,
    args: argparse.Namespace,
) -> dict[str, Any]:
    scene_path = Path(scene_row["scene_path"])
    scene_output_root = output_root / "scenes" / scene_path.stem
    runs_root = scene_output_root / "runs"
    runs_root.mkdir(parents=True, exist_ok=True)

    baseline_result = run_mode(
        analyzer,
        scene_path,
        runs_root / "context_hybrid_baseline",
        args,
        ranking_mode="context_hybrid",
        enable_relabel=False,
        relabel_mode="hard_threshold",
        score_threshold=None,
    )
    hard_result = run_mode(
        analyzer,
        scene_path,
        runs_root / "junction_mixed_priority_hard_threshold",
        args,
        ranking_mode="junction_mixed_priority",
        enable_relabel=True,
        relabel_mode="hard_threshold",
        score_threshold=None,
    )
    scored_result = run_mode(
        analyzer,
        scene_path,
        runs_root / "junction_mixed_scored_t0_66",
        args,
        ranking_mode="junction_mixed_scored",
        enable_relabel=True,
        relabel_mode="scored",
        score_threshold=args.review_threshold,
    )

    baseline_data = load_run_data(baseline_result)
    hard_data = load_run_data(hard_result)
    scored_data = load_run_data(scored_result)
    group_rows = build_group_rows(baseline_data, hard_data, scored_data)

    input_points, _ = load_point_cloud_file(str(scene_path))
    point_keys = compute_voxel_keys(input_points, args.voxel_size)
    export_group_artifacts(scene_output_root, group_rows, input_points, point_keys, args.neighborhood_radius)

    review_csv = scene_output_root / "relabeled_review_summary.csv"
    review_md = scene_output_root / "relabeled_review_summary.md"
    write_review_csv(review_csv, group_rows)
    write_review_md(review_md, scene_output_root, group_rows, args.neighborhood_radius)

    annotation_paths = write_annotation_templates(scene_output_root, group_rows)
    scene_summary_md = scene_output_root / "scene_summary.md"
    write_scene_summary_md(
        scene_summary_md,
        scene_row,
        baseline_result,
        hard_result,
        scored_result,
        group_rows,
        annotation_paths,
        args.neighborhood_radius,
    )

    groups = build_group_lookup(group_rows)
    group_stats = {group: summarize_group(rows) for group, rows in groups.items()}
    baseline_combined, baseline_planar = compute_topk_counts(baseline_result)
    hard_combined, hard_planar = compute_topk_counts(hard_result)
    scored_combined, scored_planar = compute_topk_counts(scored_result)

    return {
        "scene_name": scene_row["scene_name"],
        "scene_path": scene_row["scene_path"],
        "point_count": scene_row["point_count"],
        "selection_reason": scene_row["selection_reason"],
        "scene_output_root": str(scene_output_root),
        "review_summary_csv": str(review_csv.relative_to(output_root)),
        "review_summary_md": str(review_md.relative_to(output_root)),
        "scene_summary_md": str(scene_summary_md.relative_to(output_root)),
        "annotation_template_csv": str((scene_output_root / annotation_paths["combined_csv"]).relative_to(output_root)),
        "annotation_template_md": str((scene_output_root / annotation_paths["combined_md"]).relative_to(output_root)),
        "shared_annotation_csv": str((scene_output_root / annotation_paths["shared_relabeled_csv"]).relative_to(output_root)),
        "scored_only_annotation_csv": str((scene_output_root / annotation_paths["scored_only_relabeled_csv"]).relative_to(output_root)),
        "hard_only_annotation_csv": str((scene_output_root / annotation_paths["hard_only_relabeled_csv"]).relative_to(output_root)),
        "scored_only_combined_neighborhood": str(
            (scene_output_root / "groups" / "scored_only_relabeled" / f"combined_neighborhood_radius{args.neighborhood_radius}.pcd").relative_to(output_root)
        ),
        "hard_only_combined_neighborhood": str(
            (scene_output_root / "groups" / "hard_only_relabeled" / f"combined_neighborhood_radius{args.neighborhood_radius}.pcd").relative_to(output_root)
        ),
        "baseline_combined_topk": baseline_combined,
        "baseline_planar_topk": baseline_planar,
        "hard_combined_topk": hard_combined,
        "hard_planar_topk": hard_planar,
        "scored_combined_topk": scored_combined,
        "scored_planar_topk": scored_planar,
        "shared_count": group_stats["shared_relabeled"]["count"],
        "shared_baseline_topk_count": group_stats["shared_relabeled"]["baseline_topk_count"],
        "shared_hard_topk_count": group_stats["shared_relabeled"]["hard_topk_count"],
        "shared_scored_topk_count": group_stats["shared_relabeled"]["scored_topk_count"],
        "scored_only_count": group_stats["scored_only_relabeled"]["count"],
        "scored_only_baseline_topk_count": group_stats["scored_only_relabeled"]["baseline_topk_count"],
        "scored_only_hard_topk_count": group_stats["scored_only_relabeled"]["hard_topk_count"],
        "scored_only_scored_topk_count": group_stats["scored_only_relabeled"]["scored_topk_count"],
        "hard_only_count": group_stats["hard_only_relabeled"]["count"],
        "hard_only_baseline_topk_count": group_stats["hard_only_relabeled"]["baseline_topk_count"],
        "hard_only_hard_topk_count": group_stats["hard_only_relabeled"]["hard_topk_count"],
        "hard_only_scored_topk_count": group_stats["hard_only_relabeled"]["scored_topk_count"],
        "shared_ids": ",".join(row["voxel_id"] for row in groups["shared_relabeled"]),
        "scored_only_ids": ",".join(row["voxel_id"] for row in groups["scored_only_relabeled"]),
        "hard_only_ids": ",".join(row["voxel_id"] for row in groups["hard_only_relabeled"]),
    }


def write_scene_selection_csv(output_path: Path, scene_rows: list[dict[str, Any]]) -> None:
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["scene_name", "scene_path", "point_count", "selection_reason"],
        )
        writer.writeheader()
        for row in scene_rows:
            writer.writerow(row)


def write_scene_selection_md(output_path: Path, scene_rows: list[dict[str, Any]]) -> None:
    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# Multi-Scene Selection\n\n")
        for row in scene_rows:
            handle.write(
                f"- `{row['scene_name']}`: points={row['point_count']}, reason=`{row['selection_reason']}`, "
                f"path=`{row['scene_path']}`\n"
            )


def write_cross_scene_csv(output_path: Path, scene_results: list[dict[str, Any]]) -> None:
    fieldnames = [
        "scene_name",
        "scene_path",
        "point_count",
        "selection_reason",
        "baseline_combined_topk",
        "baseline_planar_topk",
        "hard_combined_topk",
        "hard_planar_topk",
        "scored_combined_topk",
        "scored_planar_topk",
        "shared_count",
        "shared_baseline_topk_count",
        "shared_hard_topk_count",
        "shared_scored_topk_count",
        "scored_only_count",
        "scored_only_baseline_topk_count",
        "scored_only_hard_topk_count",
        "scored_only_scored_topk_count",
        "hard_only_count",
        "hard_only_baseline_topk_count",
        "hard_only_hard_topk_count",
        "hard_only_scored_topk_count",
        "shared_ids",
        "scored_only_ids",
        "hard_only_ids",
        "scene_summary_md",
        "review_summary_csv",
        "review_summary_md",
        "annotation_template_csv",
        "annotation_template_md",
        "shared_annotation_csv",
        "scored_only_annotation_csv",
        "hard_only_annotation_csv",
        "scored_only_combined_neighborhood",
        "hard_only_combined_neighborhood",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in scene_results:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def write_cross_scene_md(output_path: Path, scene_results: list[dict[str, Any]]) -> None:
    total_shared = sum(row["shared_count"] for row in scene_results)
    total_scored_only = sum(row["scored_only_count"] for row in scene_results)
    total_hard_only = sum(row["hard_only_count"] for row in scene_results)
    total_scored_only_topk = sum(row["scored_only_scored_topk_count"] for row in scene_results)
    total_hard_only_topk = sum(row["hard_only_hard_topk_count"] for row in scene_results)

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# Junction-Like Mixed Multi-Scene Review Summary\n\n")
        handle.write("## Aggregate Counts\n\n")
        handle.write(f"- shared relabeled voxels: {total_shared}\n")
        handle.write(f"- scored-only relabeled voxels: {total_scored_only}\n")
        handle.write(f"- hard-only relabeled voxels: {total_hard_only}\n")
        handle.write(f"- scored-only top-k voxels: {total_scored_only_topk}\n")
        handle.write(f"- hard-only top-k voxels: {total_hard_only_topk}\n")

        handle.write("\n## Per-Scene Summary\n\n")
        for row in scene_results:
            handle.write(f"### {row['scene_name']}\n\n")
            handle.write(f"- scene path: `{row['scene_path']}`\n")
            handle.write(
                f"- top-k: baseline combined={row['baseline_combined_topk']}, planar={row['baseline_planar_topk']}; "
                f"hard combined={row['hard_combined_topk']}, planar={row['hard_planar_topk']}; "
                f"scored combined={row['scored_combined_topk']}, planar={row['scored_planar_topk']}\n"
            )
            handle.write(
                f"- groups: shared={row['shared_count']} (scored_topk={row['shared_scored_topk_count']}), "
                f"scored_only={row['scored_only_count']} (scored_topk={row['scored_only_scored_topk_count']}), "
                f"hard_only={row['hard_only_count']} (hard_topk={row['hard_only_hard_topk_count']})\n"
            )
            handle.write(f"- shared ids: {row['shared_ids'] or '(none)'}\n")
            handle.write(f"- scored-only ids: {row['scored_only_ids'] or '(none)'}\n")
            handle.write(f"- hard-only ids: {row['hard_only_ids'] or '(none)'}\n")
            handle.write("- inspect first:\n")
            handle.write(f"  - `{row['scored_only_combined_neighborhood']}`\n")
            handle.write(f"  - `{row['scored_only_annotation_csv']}`\n")
            handle.write(f"  - `{row['hard_only_combined_neighborhood']}`\n")
            handle.write(f"  - `{row['hard_only_annotation_csv']}`\n")
            handle.write(f"  - `{row['scene_summary_md']}`\n")
            handle.write(f"- annotation template: `{row['annotation_template_csv']}`\n\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run multi-scene scored relabel review and annotation export.")
    parser.add_argument("--input-root", default="./prev/BALM/datas/benchmark_realworld")
    parser.add_argument("--scene-names", default="")
    parser.add_argument("--output-root", default="results/context_refinement_scored_multiscene_review")
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
    input_root = (repo_root / args.input_root).resolve() if not Path(args.input_root).is_absolute() else Path(args.input_root)
    output_root = (repo_root / args.output_root).resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    analyzer = repo_root / "build" / "ges_voxel_mapping_cpp" / "voxel_morphology_analyzer"
    if not analyzer.exists():
        subprocess.run([str(repo_root / "scripts" / "build_offline.sh")], check=True)

    scene_rows = choose_scene_rows(input_root, parse_scene_names(args.scene_names))
    write_scene_selection_csv(output_root / "scene_selection.csv", scene_rows)
    write_scene_selection_md(output_root / "scene_selection.md", scene_rows)

    scene_results = []
    for scene_row in scene_rows:
        scene_results.append(process_scene(analyzer, scene_row, output_root, args))

    write_cross_scene_csv(output_root / "cross_scene_summary.csv", scene_results)
    write_cross_scene_md(output_root / "cross_scene_summary.md", scene_results)

    print(f"Wrote {output_root / 'scene_selection.csv'}")
    print(f"Wrote {output_root / 'scene_selection.md'}")
    print(f"Wrote {output_root / 'cross_scene_summary.csv'}")
    print(f"Wrote {output_root / 'cross_scene_summary.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
