#!/usr/bin/env python3
import argparse
import csv
import math
import os
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    config.setdefault("input_path", "")
    config.setdefault("output_dir", "results/voxel_morphology")
    config.setdefault("input_mode", "auto")
    config.setdefault("voxel_size", 1.0)
    config.setdefault("min_points_per_voxel", 20)
    config.setdefault("max_points", -1)
    config.setdefault("max_voxels", -1)
    config.setdefault("gaussian_regularization", 1e-3)
    config.setdefault("shape_exponent", 1.2)
    config.setdefault("axis_scale_quantile", 0.9)
    config.setdefault("axis_scale_min", 0.05)
    config.setdefault("save_top_k", 200)
    config.setdefault("export_interesting_voxels", True)
    config.setdefault("export_top_k_pcd", 30)
    return config


def normalize_mode(mode: str) -> str:
    mode = (mode or "auto").lower()
    if mode not in {"auto", "single", "aggregate", "batch"}:
        raise ValueError(f"Unsupported mode: {mode}")
    return mode


def collect_input_files(input_path: str) -> List[str]:
    path = Path(input_path)
    if not path.exists():
        raise FileNotFoundError(f"Input path does not exist: {input_path}")
    if path.is_file():
        if path.suffix.lower() != ".pcd":
            raise ValueError(f"Only .pcd input is supported: {input_path}")
        return [str(path)]
    files = sorted(str(p) for p in path.iterdir() if p.is_file() and p.suffix.lower() == ".pcd")
    if not files:
        raise FileNotFoundError(f"No .pcd files found under: {input_path}")
    return files


def pcd_numpy_dtype(fields, sizes, types, counts):
    type_map = {
        ("F", 4): np.float32,
        ("F", 8): np.float64,
        ("I", 1): np.int8,
        ("I", 2): np.int16,
        ("I", 4): np.int32,
        ("I", 8): np.int64,
        ("U", 1): np.uint8,
        ("U", 2): np.uint16,
        ("U", 4): np.uint32,
        ("U", 8): np.uint64,
    }
    dtype_fields = []
    field_names = []
    for field, size, typ, count in zip(fields, sizes, types, counts):
        key = (typ, size)
        if key not in type_map:
            raise ValueError(f"Unsupported PCD field type: {key}")
        base_dtype = type_map[key]
        for idx in range(count):
            name = field if count == 1 else f"{field}_{idx}"
            dtype_fields.append((name, base_dtype))
            field_names.append(name)
    return np.dtype(dtype_fields), field_names


def parse_pcd(path: str) -> Tuple[np.ndarray, str]:
    header = {}
    header_lines = []
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"Invalid PCD without DATA line: {path}")
            decoded = line.decode("ascii", errors="ignore").strip()
            header_lines.append(decoded)
            if decoded.startswith("#") or not decoded:
                continue
            key, *rest = decoded.split()
            header[key.upper()] = rest
            if key.upper() == "DATA":
                data_type = rest[0].lower()
                break
        fields = header["FIELDS"]
        sizes = list(map(int, header["SIZE"]))
        types = header["TYPE"]
        counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
        points = int(header.get("POINTS", [header["WIDTH"][0]])[0])
        dtype, expanded_fields = pcd_numpy_dtype(fields, sizes, types, counts)
        if data_type == "binary":
            data = np.fromfile(f, dtype=dtype, count=points)
        elif data_type == "ascii":
            data = np.loadtxt(f, dtype=dtype, ndmin=1)
        else:
            raise ValueError(f"Unsupported PCD DATA type: {data_type}")
    return data, ";".join(expanded_fields)


def load_point_cloud_file(path: str):
    data, field_string = parse_pcd(path)
    names = set(data.dtype.names or [])
    if not {"x", "y", "z"}.issubset(names):
        raise ValueError(f"PCD must contain x/y/z fields: {path}")
    intensity_name = "intensity" if "intensity" in names else ("i" if "i" in names else None)
    xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float64, copy=False)
    intensity = (
        np.asarray(data[intensity_name], dtype=np.float64)
        if intensity_name is not None
        else np.zeros((xyz.shape[0],), dtype=np.float64)
    )
    finite_mask = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite_mask]
    intensity = intensity[finite_mask]
    points = np.concatenate([xyz, intensity[:, None]], axis=1)
    desc = f"{path} | points={points.shape[0]} | fields={field_string} | intensity={'yes' if intensity_name else 'no'}"
    return points, desc


def maybe_downsample(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or points.shape[0] <= max_points:
        return points
    stride = int(math.ceil(points.shape[0] / float(max_points)))
    return points[::stride]


def compute_quantile(values: np.ndarray, quantile: float) -> float:
    if values.size == 0:
        return 0.0
    q = min(max(float(quantile), 0.0), 1.0)
    sorted_values = np.sort(values, axis=0)
    index = int(math.floor(q * float(sorted_values.shape[0] - 1)))
    return sorted_values[index]


def format_float(value: float) -> str:
    return f"{float(value):.6f}"


def compute_voxel_metrics(points_xyz: np.ndarray, voxel_key: Tuple[int, int, int], cfg: dict) -> dict:
    num_points = points_xyz.shape[0]
    mean = points_xyz.mean(axis=0)
    centered = points_xyz - mean
    covariance = np.cov(centered.T, bias=False) if num_points >= 2 else np.zeros((3, 3))
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = np.maximum(eigenvalues[order], 0.0)
    eigenvectors = eigenvectors[:, order]

    l1 = max(float(eigenvalues[0]), 1e-9)
    l2 = max(float(eigenvalues[1]), 0.0)
    l3 = max(float(eigenvalues[2]), 0.0)
    linearity = (l1 - l2) / l1
    planarity = (l2 - l3) / l1
    scattering = l3 / l1
    anisotropy = (l1 - l3) / l1
    omnivariance = max(l1 * l2 * l3, 0.0) ** (1.0 / 3.0)

    ratio21 = l2 / l1
    ratio31 = l3 / l1
    if num_points < 5 or l1 < 1e-8:
        label = "degenerate"
    elif ratio21 < 0.25 and ratio31 < 0.08:
        label = "linear"
    elif ratio21 >= 0.25 and ratio31 < 0.12:
        label = "planar"
    elif ratio31 > 0.30:
        label = "volumetric"
    else:
        label = "corner_like"

    center = (np.array(voxel_key, dtype=np.float64) + 0.5) * float(cfg["voxel_size"])
    covariance_reg = covariance + np.eye(3) * float(cfg["gaussian_regularization"])
    covariance_inv = np.linalg.inv(covariance_reg)
    deltas = points_xyz - mean
    avg_mahal = float(np.mean(np.einsum("ni,ij,nj->n", deltas, covariance_inv, deltas)))
    center_mahal = float((center - mean).T @ covariance_inv @ (center - mean))
    avg_euclidean = float(np.mean(np.linalg.norm(deltas, axis=1)))
    center_advantage = avg_mahal - center_mahal

    local_points = (eigenvectors.T @ centered.T).T
    axis_abs = np.abs(local_points)
    axis_scales = np.array([
        max(compute_quantile(axis_abs[:, 0], cfg["axis_scale_quantile"]), float(cfg["axis_scale_min"])),
        max(compute_quantile(axis_abs[:, 1], cfg["axis_scale_quantile"]), float(cfg["axis_scale_min"])),
        max(compute_quantile(axis_abs[:, 2], cfg["axis_scale_quantile"]), float(cfg["axis_scale_min"])),
    ], dtype=np.float64)
    p = max(float(cfg["shape_exponent"]), 0.25)
    radii = np.sum((np.abs(local_points) / axis_scales) ** p, axis=1) ** (1.0 / p)
    avg_surface_radius = float(np.mean(radii))
    avg_surface_residual = float(np.mean(np.abs(radii - 1.0)))
    center_local = eigenvectors.T @ (center - mean)
    center_radius = float(np.sum((np.abs(center_local) / axis_scales) ** p) ** (1.0 / p))
    center_surface_residual = abs(center_radius - 1.0)
    center_penalty = center_surface_residual - avg_surface_residual

    return {
        "key": voxel_key,
        "center": center,
        "num_points": num_points,
        "label": label,
        "eigenvalues": eigenvalues,
        "linearity": linearity,
        "planarity": planarity,
        "scattering": scattering,
        "anisotropy": anisotropy,
        "omnivariance": omnivariance,
        "gaussian_avg_mahalanobis2": avg_mahal,
        "gaussian_center_mahalanobis2": center_mahal,
        "gaussian_avg_euclidean": avg_euclidean,
        "gaussian_center_advantage": center_advantage,
        "shell_avg_residual": avg_surface_residual,
        "shell_center_residual": center_surface_residual,
        "shell_avg_radius": avg_surface_radius,
        "shell_center_penalty": center_penalty,
    }


def write_xyz_pcd(path: Path, points: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="ascii") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {points.shape[0]}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {points.shape[0]}\n")
        f.write("DATA ascii\n")
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f}\n")


def save_results(metrics: List[dict], cfg: dict, loaded_files: List[str], input_points: np.ndarray):
    output_dir = Path(cfg["output_dir"])
    output_dir.mkdir(parents=True, exist_ok=True)

    metrics_by_key = sorted(
        metrics,
        key=lambda m: (m["key"][0], m["key"][1], m["key"][2]),
    )

    metrics_sorted = sorted(
        metrics,
        key=lambda m: (
            -(m["gaussian_center_advantage"] + m["shell_center_penalty"]),
            m["key"][0],
            m["key"][1],
            m["key"][2],
        )
    )

    with open(output_dir / "voxel_metrics.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow([
            "vx", "vy", "vz", "center_x", "center_y", "center_z", "num_points", "label",
            "eig1", "eig2", "eig3", "linearity", "planarity", "scattering", "anisotropy", "omnivariance",
            "gaussian_avg_mahalanobis2", "gaussian_center_mahalanobis2", "gaussian_avg_euclidean", "gaussian_center_advantage",
            "shell_avg_residual", "shell_center_residual", "shell_avg_radius", "shell_center_penalty",
        ])
        for m in metrics_by_key:
            writer.writerow([
                m["key"][0], m["key"][1], m["key"][2],
                format_float(m["center"][0]), format_float(m["center"][1]), format_float(m["center"][2]),
                m["num_points"], m["label"],
                format_float(m["eigenvalues"][0]), format_float(m["eigenvalues"][1]), format_float(m["eigenvalues"][2]),
                format_float(m["linearity"]), format_float(m["planarity"]), format_float(m["scattering"]),
                format_float(m["anisotropy"]), format_float(m["omnivariance"]),
                format_float(m["gaussian_avg_mahalanobis2"]), format_float(m["gaussian_center_mahalanobis2"]),
                format_float(m["gaussian_avg_euclidean"]), format_float(m["gaussian_center_advantage"]),
                format_float(m["shell_avg_residual"]), format_float(m["shell_center_residual"]),
                format_float(m["shell_avg_radius"]), format_float(m["shell_center_penalty"]),
            ])

    with open(output_dir / "interesting_voxels.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["rank", "vx", "vy", "vz", "label", "num_points", "gaussian_center_advantage", "shell_center_penalty", "total_score"])
        for rank, m in enumerate(metrics_sorted[: int(cfg["save_top_k"])]):
            score = m["gaussian_center_advantage"] + m["shell_center_penalty"]
            writer.writerow([
                rank, m["key"][0], m["key"][1], m["key"][2], m["label"], m["num_points"],
                format_float(m["gaussian_center_advantage"]),
                format_float(m["shell_center_penalty"]),
                format_float(score),
            ])

    label_counts: Dict[str, int] = {}
    for m in metrics_by_key:
        label_counts[m["label"]] = label_counts.get(m["label"], 0) + 1

    with open(output_dir / "summary.txt", "w", encoding="utf-8") as f:
        f.write(f"input_points: {input_points.shape[0]}\n")
        f.write(f"analyzed_voxels: {len(metrics)}\n")
        f.write(f"input_mode: {cfg['input_mode']}\n")
        f.write(f"voxel_size: {format_float(cfg['voxel_size'])}\n")
        f.write(f"min_points_per_voxel: {cfg['min_points_per_voxel']}\n")
        f.write(f"shape_exponent: {format_float(cfg['shape_exponent'])}\n")
        f.write(f"axis_scale_quantile: {format_float(cfg['axis_scale_quantile'])}\n")
        f.write(
            f"average_gaussian_center_advantage: "
            f"{format_float(np.mean([m['gaussian_center_advantage'] for m in metrics])) if metrics else format_float(0.0)}\n"
        )
        f.write(
            f"average_shell_center_penalty: "
            f"{format_float(np.mean([m['shell_center_penalty'] for m in metrics])) if metrics else format_float(0.0)}\n"
        )
        f.write("label_counts:\n")
        for key, value in sorted(label_counts.items()):
            f.write(f"  {key}: {value}\n")

    with open(output_dir / "loaded_files.txt", "w", encoding="utf-8") as f:
        for line in loaded_files:
            f.write(line + "\n")

    if cfg.get("export_interesting_voxels", True):
        voxel_dir = output_dir / "interesting_voxels"
        voxel_dir.mkdir(parents=True, exist_ok=True)
        top_metrics = metrics_sorted[: min(int(cfg["export_top_k_pcd"]), int(cfg["save_top_k"]))]
        selected_keys = {tuple(m["key"]): idx for idx, m in enumerate(top_metrics)}

        voxel_key_all = np.floor(input_points[:, :3] / float(cfg["voxel_size"])).astype(int)
        combined_points = []
        manifest_rows = []
        for rank, m in enumerate(top_metrics):
            key = tuple(m["key"])
            mask = np.all(voxel_key_all == np.array(key), axis=1)
            voxel_points = input_points[mask]
            combined_points.append(voxel_points)
            filename = f"rank_{rank}_vx_{key[0]}_vy_{key[1]}_vz_{key[2]}.pcd"
            write_xyz_pcd(voxel_dir / filename, voxel_points)
            manifest_rows.append([rank, filename, key[0], key[1], key[2], m["label"], m["num_points"], format_float(m["gaussian_center_advantage"] + m["shell_center_penalty"])])

        combined = np.concatenate(combined_points, axis=0) if combined_points else np.zeros((0, 4), dtype=np.float64)
        write_xyz_pcd(voxel_dir / "interesting_voxels_combined.pcd", combined)
        with open(voxel_dir / "manifest.csv", "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f, lineterminator="\n")
            writer.writerow(["rank", "filename", "vx", "vy", "vz", "label", "num_points", "total_score"])
            writer.writerows(manifest_rows)


def analyze_input(input_path: str, output_dir: str, cfg: dict):
    files = collect_input_files(input_path)
    loaded_files = []
    clouds = []
    for file in files:
        points, desc = load_point_cloud_file(file)
        clouds.append(points)
        loaded_files.append(desc)
    all_points = np.concatenate(clouds, axis=0) if clouds else np.zeros((0, 4), dtype=np.float64)
    all_points = maybe_downsample(all_points, int(cfg["max_points"]))
    voxel_size = float(cfg["voxel_size"])
    voxel_keys = np.floor(all_points[:, :3] / voxel_size).astype(int)
    voxel_map: Dict[Tuple[int, int, int], List[int]] = {}
    for idx, key in enumerate(map(tuple, voxel_keys)):
        voxel_map.setdefault(key, []).append(idx)

    metrics = []
    processed = 0
    for key, indices in voxel_map.items():
        if len(indices) < int(cfg["min_points_per_voxel"]):
            continue
        metrics.append(compute_voxel_metrics(all_points[np.array(indices), :3], key, cfg))
        processed += 1
        if int(cfg["max_voxels"]) > 0 and processed >= int(cfg["max_voxels"]):
            break

    run_cfg = dict(cfg)
    run_cfg["output_dir"] = output_dir
    save_results(metrics, run_cfg, loaded_files, all_points)
    print(f"[python-fallback-done] input={input_path} points={all_points.shape[0]} voxels={len(metrics)} output={output_dir}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--mode", default="auto")
    parser.add_argument("--output-is-final", action="store_true")
    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg["input_path"] = args.input
    cfg["output_dir"] = args.output
    cfg["input_mode"] = normalize_mode(args.mode)

    input_path = Path(cfg["input_path"])
    effective_mode = cfg["input_mode"]
    if effective_mode == "auto":
        effective_mode = "aggregate" if input_path.is_dir() else "single"

    if effective_mode == "single":
        if input_path.is_dir():
            raise ValueError("single mode expects a file")
        output_dir = Path(cfg["output_dir"]) if args.output_is_final else Path(cfg["output_dir"]) / input_path.stem
        analyze_input(str(input_path), str(output_dir), cfg)
        return

    if effective_mode == "aggregate":
        analyze_input(str(input_path), str(Path(cfg["output_dir"])), cfg)
        return

    if effective_mode == "batch":
        if input_path.is_file():
            output_dir = Path(cfg["output_dir"]) if args.output_is_final else Path(cfg["output_dir"]) / input_path.stem
            analyze_input(str(input_path), str(output_dir), cfg)
            return
        for file in collect_input_files(str(input_path)):
            analyze_input(file, str(Path(cfg["output_dir"]) / Path(file).stem), cfg)
        print(f"[python-fallback-batch-done] input_dir={input_path} output_root={cfg['output_dir']}")
        return

    raise ValueError(f"Unexpected mode: {effective_mode}")


if __name__ == "__main__":
    main()
