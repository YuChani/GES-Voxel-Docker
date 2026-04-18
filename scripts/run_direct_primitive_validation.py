#!/usr/bin/env python3

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run Stage-1 direct primitive voxel validation using the C++ offline executable."
    )
    parser.add_argument("--input", required=True, help="Input .pcd file or directory")
    parser.add_argument(
        "--output-root",
        default="./results/direct_primitive_validation",
        help="Root directory for Stage-1 outputs",
    )
    parser.add_argument("--run-name", default="stage1_run", help="Subdirectory name under output-root")
    parser.add_argument("--voxel-size", type=float, default=1.0)
    parser.add_argument("--min-points-per-voxel", type=int, default=15)
    parser.add_argument("--max-points", type=int, default=-1)
    parser.add_argument("--max-voxels", type=int, default=12)
    parser.add_argument("--per-category-limit", type=int, default=4)
    parser.add_argument("--shell-shape-exponent", type=float, default=1.0)
    parser.add_argument("--shell-axis-scale-quantile", type=float, default=0.90)
    parser.add_argument("--gaussian-regularization", type=float, default=1e-3)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    executable = repo_root / "build" / "ges_voxel_mapping_cpp" / "voxel_primitive_comparison"
    if not executable.exists():
        raise SystemExit(
            "Missing voxel_primitive_comparison executable. Run ./scripts/build_offline.sh first."
        )

    output_dir = (repo_root / args.output_root / args.run_name).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    command = [
        str(executable),
        "--input",
        args.input,
        "--output",
        str(output_dir),
        "--voxel-size",
        str(args.voxel_size),
        "--min-points-per-voxel",
        str(args.min_points_per_voxel),
        "--max-points",
        str(args.max_points),
        "--max-voxels",
        str(args.max_voxels),
        "--per-category-limit",
        str(args.per_category_limit),
        "--gaussian-regularization",
        str(args.gaussian_regularization),
        "--shell-axis-scale-quantile",
        str(args.shell_axis_scale_quantile),
        "--shell-shape-exponent",
        str(args.shell_shape_exponent),
    ]

    print("Running:", " ".join(command), flush=True)
    subprocess.run(command, cwd=repo_root, check=True)
    print("Output directory:", output_dir, flush=True)
    print("Summary:", output_dir / "summary.md", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
