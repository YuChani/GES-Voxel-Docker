#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import itertools
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


DEFAULT_VOXEL_SIZES = [0.8, 1.0, 1.2]
DEFAULT_MIN_POINTS = [15, 20]
DEFAULT_SHAPE_EXPONENTS = [1.0, 1.4]
DEFAULT_AXIS_SCALE_QUANTILES = [0.85, 0.95]
DEFAULT_RANKING_MODES = ["score_only", "corner_priority", "nonplanar_priority"]
BASELINE_PARAMS = {
    "voxel_size": 1.0,
    "min_points_per_voxel": 20,
    "shape_exponent": 1.2,
    "axis_scale_quantile": 0.9,
}


def parse_float_list(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def parse_int_list(text: str) -> list[int]:
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def parse_text_list(text: str) -> list[str]:
    return [item.strip() for item in text.split(",") if item.strip()]


def format_float(value: float) -> str:
    return f"{value:.3f}".rstrip("0").rstrip(".")


def slugify_float(value: float) -> str:
    return format_float(value).replace("-", "m").replace(".", "p")


def run_slug(params: dict[str, Any]) -> str:
    return (
        f"vs_{slugify_float(params['voxel_size'])}"
        f"__mp_{params['min_points_per_voxel']}"
        f"__se_{slugify_float(params['shape_exponent'])}"
        f"__aq_{slugify_float(params['axis_scale_quantile'])}"
        f"__mode_{params['ranking_mode']}"
    )


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


def flatten_counts(prefix: str, labels: list[str], counts: dict[str, int]) -> dict[str, Any]:
    flattened: dict[str, Any] = {}
    total = sum(counts.values())
    for label in labels:
        count = counts.get(label, 0)
        flattened[f"{prefix}_{label}_count"] = count
        flattened[f"{prefix}_{label}_fraction"] = 0.0 if total == 0 else count / total
    return flattened


def build_runs(
    voxel_sizes: list[float],
    min_points_values: list[int],
    shape_exponents: list[float],
    axis_scale_quantiles: list[float],
    ranking_modes: list[str],
    include_baseline: bool,
) -> list[dict[str, Any]]:
    run_specs: list[dict[str, Any]] = []
    seen: set[tuple[Any, ...]] = set()

    def add_run(spec: dict[str, Any]) -> None:
        key = (
            spec["voxel_size"],
            spec["min_points_per_voxel"],
            spec["shape_exponent"],
            spec["axis_scale_quantile"],
            spec["ranking_mode"],
        )
        if key in seen:
            return
        seen.add(key)
        run_specs.append(spec)

    for voxel_size, min_points, shape_exponent, axis_scale_quantile, ranking_mode in itertools.product(
        voxel_sizes,
        min_points_values,
        shape_exponents,
        axis_scale_quantiles,
        ranking_modes,
    ):
        add_run(
            {
                "voxel_size": voxel_size,
                "min_points_per_voxel": min_points,
                "shape_exponent": shape_exponent,
                "axis_scale_quantile": axis_scale_quantile,
                "ranking_mode": ranking_mode,
                "is_baseline": False,
            }
        )

    if include_baseline:
        for ranking_mode in ranking_modes:
            add_run({**BASELINE_PARAMS, "ranking_mode": ranking_mode, "is_baseline": True})

    return run_specs


def write_summary_csv(output_path: Path, rows: list[dict[str, Any]], labels: list[str]) -> None:
    fieldnames = [
        "run_id",
        "output_dir",
        "is_baseline",
        "voxel_size",
        "min_points_per_voxel",
        "shape_exponent",
        "axis_scale_quantile",
        "ranking_mode",
        "analyzed_voxels",
        "interesting_top_k",
        "average_gaussian_center_advantage",
        "average_shell_center_penalty",
        "average_morphology_filter_penalty",
        "corner_minus_planar_topk",
    ]
    for prefix in ("label", "topk"):
        for label in labels:
            fieldnames.append(f"{prefix}_{label}_count")
            fieldnames.append(f"{prefix}_{label}_fraction")

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            flattened = {key: row[key] for key in fieldnames if key in row}
            flattened.update(flatten_counts("label", labels, row["label_counts"]))
            flattened.update(flatten_counts("topk", labels, row["topk_label_counts"]))
            writer.writerow(flattened)


def write_mode_comparison_csv(output_path: Path, rows: list[dict[str, Any]]) -> None:
    grouped: dict[tuple[Any, ...], dict[str, dict[str, Any]]] = defaultdict(dict)
    for row in rows:
        key = (
            row["voxel_size"],
            row["min_points_per_voxel"],
            row["shape_exponent"],
            row["axis_scale_quantile"],
        )
        grouped[key][row["ranking_mode"]] = row

    fieldnames = [
        "voxel_size",
        "min_points_per_voxel",
        "shape_exponent",
        "axis_scale_quantile",
    ]
    for mode in DEFAULT_RANKING_MODES:
        fieldnames.extend(
            [
                f"{mode}_corner_like_fraction",
                f"{mode}_planar_fraction",
                f"{mode}_corner_minus_planar",
            ]
        )

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for key in sorted(grouped):
            row: dict[str, Any] = {
                "voxel_size": key[0],
                "min_points_per_voxel": key[1],
                "shape_exponent": key[2],
                "axis_scale_quantile": key[3],
            }
            for mode in DEFAULT_RANKING_MODES:
                item = grouped[key].get(mode)
                if item is None:
                    row[f"{mode}_corner_like_fraction"] = ""
                    row[f"{mode}_planar_fraction"] = ""
                    row[f"{mode}_corner_minus_planar"] = ""
                    continue
                topk = item["topk_label_counts"]
                topk_total = max(sum(topk.values()), 1)
                row[f"{mode}_corner_like_fraction"] = topk.get("corner_like", 0) / topk_total
                row[f"{mode}_planar_fraction"] = topk.get("planar", 0) / topk_total
                row[f"{mode}_corner_minus_planar"] = topk.get("corner_like", 0) - topk.get("planar", 0)
            writer.writerow(row)


def write_markdown_summary(output_path: Path, rows: list[dict[str, Any]]) -> None:
    def score(row: dict[str, Any]) -> tuple[float, float, int, int]:
        topk = row["topk_label_counts"]
        topk_total = max(sum(topk.values()), 1)
        corner_fraction = topk.get("corner_like", 0) / topk_total
        planar_fraction = topk.get("planar", 0) / topk_total
        margin = topk.get("corner_like", 0) - topk.get("planar", 0)
        return (corner_fraction, -planar_fraction, margin, row["analyzed_voxels"])

    ordered = sorted(rows, key=score, reverse=True)
    baseline_rows = [row for row in rows if row["is_baseline"]]
    per_mode_best = {}
    for mode in DEFAULT_RANKING_MODES:
        candidates = [row for row in rows if row["ranking_mode"] == mode]
        if candidates:
            per_mode_best[mode] = max(candidates, key=score)

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# Parameter Sweep Summary\n\n")
        handle.write(f"Total runs: {len(rows)}\n\n")
        if baseline_rows:
            handle.write("## Baseline\n\n")
            for row in sorted(baseline_rows, key=lambda item: DEFAULT_RANKING_MODES.index(item["ranking_mode"])):
                topk = row["topk_label_counts"]
                topk_total = max(sum(topk.values()), 1)
                handle.write(
                    f"- `{row['ranking_mode']}`: corner_like={topk.get('corner_like', 0)}/{topk_total} "
                    f"({topk.get('corner_like', 0) / topk_total:.3f}), "
                    f"planar={topk.get('planar', 0)}/{topk_total} "
                    f"({topk.get('planar', 0) / topk_total:.3f})\n"
                )
            handle.write("\n")

        handle.write("## Best Overall\n\n")
        for row in ordered[:5]:
            topk = row["topk_label_counts"]
            topk_total = max(sum(topk.values()), 1)
            handle.write(
                f"- `{row['run_id']}`: mode={row['ranking_mode']}, "
                f"corner_like={topk.get('corner_like', 0)}/{topk_total} "
                f"({topk.get('corner_like', 0) / topk_total:.3f}), "
                f"planar={topk.get('planar', 0)}/{topk_total} "
                f"({topk.get('planar', 0) / topk_total:.3f}), "
                f"analyzed_voxels={row['analyzed_voxels']}\n"
            )
        handle.write("\n")

        handle.write("## Best Per Mode\n\n")
        for mode in DEFAULT_RANKING_MODES:
            row = per_mode_best.get(mode)
            if row is None:
                continue
            topk = row["topk_label_counts"]
            topk_total = max(sum(topk.values()), 1)
            handle.write(
                f"- `{mode}`: `{row['run_id']}` with corner_like={topk.get('corner_like', 0)}/{topk_total} "
                f"({topk.get('corner_like', 0) / topk_total:.3f}) and planar={topk.get('planar', 0)}/{topk_total} "
                f"({topk.get('planar', 0) / topk_total:.3f})\n"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the C++ voxel morphology parameter sweep.")
    parser.add_argument("--input", required=True, help="Input PCD file")
    parser.add_argument("--output-root", default="results/parameter_sweep", help="Sweep output directory")
    parser.add_argument("--voxel-sizes", default="0.8,1.0,1.2")
    parser.add_argument("--min-points", default="15,20")
    parser.add_argument("--shape-exponents", default="1.0,1.4")
    parser.add_argument("--axis-scale-quantiles", default="0.85,0.95")
    parser.add_argument("--ranking-modes", default="score_only,corner_priority,nonplanar_priority")
    parser.add_argument("--top-k", type=int, default=200)
    parser.add_argument("--include-baseline", action="store_true")
    parser.add_argument("--export-interesting-voxels", action="store_true")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    input_path = (repo_root / args.input).resolve() if not Path(args.input).is_absolute() else Path(args.input)
    output_root = (repo_root / args.output_root).resolve()
    runs_root = output_root / "runs"
    output_root.mkdir(parents=True, exist_ok=True)
    runs_root.mkdir(parents=True, exist_ok=True)

    analyzer = repo_root / "build" / "ges_voxel_mapping_cpp" / "voxel_morphology_analyzer"
    if not analyzer.exists():
        subprocess.run([str(repo_root / "scripts" / "build_offline.sh")], check=True)

    run_specs = build_runs(
        voxel_sizes=parse_float_list(args.voxel_sizes),
        min_points_values=parse_int_list(args.min_points),
        shape_exponents=parse_float_list(args.shape_exponents),
        axis_scale_quantiles=parse_float_list(args.axis_scale_quantiles),
        ranking_modes=parse_text_list(args.ranking_modes),
        include_baseline=args.include_baseline,
    )

    rows: list[dict[str, Any]] = []
    all_labels: set[str] = set()

    for index, spec in enumerate(run_specs, start=1):
        run_id = run_slug(spec)
        run_output_dir = runs_root / run_id
        command = [
            str(analyzer),
            "--input",
            str(input_path),
            "--mode",
            "single",
            "--output",
            str(run_output_dir),
            "--voxel-size",
            str(spec["voxel_size"]),
            "--min-points-per-voxel",
            str(spec["min_points_per_voxel"]),
            "--shape-exponent",
            str(spec["shape_exponent"]),
            "--axis-scale-quantile",
            str(spec["axis_scale_quantile"]),
            "--ranking-mode",
            spec["ranking_mode"],
            "--save-top-k",
            str(args.top_k),
            "--export-interesting-voxels",
            "true" if args.export_interesting_voxels else "false",
            "--export-top-k-pcd",
            "0",
        ]
        print(f"[{index}/{len(run_specs)}] {' '.join(command)}", flush=True)
        subprocess.run(command, check=True)

        summary = parse_summary(run_output_dir / "summary.txt")
        topk_counts, topk_total = parse_interesting_labels(run_output_dir / "interesting_voxels.csv")
        label_counts = Counter(summary.get("label_counts", {}))

        all_labels.update(label_counts.keys())
        all_labels.update(topk_counts.keys())

        row = {
            "run_id": run_id,
            "output_dir": str(run_output_dir),
            "is_baseline": spec["is_baseline"],
            "voxel_size": spec["voxel_size"],
            "min_points_per_voxel": spec["min_points_per_voxel"],
            "shape_exponent": spec["shape_exponent"],
            "axis_scale_quantile": spec["axis_scale_quantile"],
            "ranking_mode": spec["ranking_mode"],
            "analyzed_voxels": int(summary["analyzed_voxels"]),
            "interesting_top_k": topk_total,
            "average_gaussian_center_advantage": float(summary["average_gaussian_center_advantage"]),
            "average_shell_center_penalty": float(summary["average_shell_center_penalty"]),
            "average_morphology_filter_penalty": float(summary.get("average_morphology_filter_penalty", 0.0)),
            "corner_minus_planar_topk": topk_counts.get("corner_like", 0) - topk_counts.get("planar", 0),
            "label_counts": dict(label_counts),
            "topk_label_counts": dict(topk_counts),
        }
        rows.append(row)

    labels = sorted(all_labels)
    write_summary_csv(output_root / "sweep_summary.csv", rows, labels)
    write_mode_comparison_csv(output_root / "mode_comparison.csv", rows)
    write_markdown_summary(output_root / "summary.md", rows)

    print(f"Wrote {output_root / 'sweep_summary.csv'}")
    print(f"Wrote {output_root / 'mode_comparison.csv'}")
    print(f"Wrote {output_root / 'summary.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
