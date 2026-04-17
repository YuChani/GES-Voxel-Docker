#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import textwrap
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from voxel_morphology_fallback import load_point_cloud_file

GROUP_NAMES = ("scored_only_relabeled", "hard_only_relabeled", "shared_relabeled")
VIEW_NAMES = ("isometric", "front", "top")
HUMAN_LABEL_OPTIONS = (
    "junction_like_mixed",
    "planar_boundary",
    "ambiguous",
    "noise_or_bad_case",
)
WHITE = (255, 255, 255, 255)
TEXT = (20, 20, 20, 255)
GRID = (220, 220, 220, 255)
NEIGHBOR_COLOR = (150, 150, 160, 150)
VOXEL_COLOR = (236, 108, 44, 235)
SHARED_COLOR = (75, 133, 236, 235)
EMPTY_BG = (246, 246, 246, 255)


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def safe_int(text: str) -> int | None:
    if text == "":
        return None
    return int(text)


def scene_sort_key(scene_name: str) -> tuple[int, str]:
    stem = Path(scene_name).stem
    digits = "".join(char for char in stem if char.isdigit())
    return (int(digits) if digits else 0, scene_name)


def choose_font(size: int = 14) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except Exception:
        return ImageFont.load_default()


def load_xyz_points(path: Path) -> np.ndarray:
    points, _ = load_point_cloud_file(str(path))
    return np.asarray(points[:, :3], dtype=np.float64)


def projection_basis(view_name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if view_name == "top":
        view_dir = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        right = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    elif view_name == "front":
        view_dir = np.array([0.0, -1.0, 0.0], dtype=np.float64)
        right = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    elif view_name == "isometric":
        view_dir = np.array([1.0, -1.0, 1.0], dtype=np.float64)
        view_dir /= np.linalg.norm(view_dir)
        world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        right = np.cross(view_dir, world_up)
        right /= np.linalg.norm(right)
        up = np.cross(right, view_dir)
        up /= np.linalg.norm(up)
    else:
        raise ValueError(f"Unsupported view: {view_name}")
    return right, up, view_dir


def project_points(points: np.ndarray, view_name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    right, up, view_dir = projection_basis(view_name)
    x = points @ right
    y = points @ up
    depth = points @ view_dir
    return x, y, depth


def compute_bounds(reference_points: np.ndarray, view_name: str) -> tuple[float, float, float, float]:
    if reference_points.size == 0:
        return (-1.0, 1.0, -1.0, 1.0)
    x, y, _ = project_points(reference_points, view_name)
    min_x, max_x = float(np.min(x)), float(np.max(x))
    min_y, max_y = float(np.min(y)), float(np.max(y))
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    margin = 0.12 * max(span_x, span_y, 1.0)
    return (min_x - margin, max_x + margin, min_y - margin, max_y + margin)


def map_to_canvas(
    x: np.ndarray,
    y: np.ndarray,
    bounds: tuple[float, float, float, float],
    size: int,
) -> tuple[np.ndarray, np.ndarray]:
    min_x, max_x, min_y, max_y = bounds
    span_x = max(max_x - min_x, 1e-9)
    span_y = max(max_y - min_y, 1e-9)
    px = (x - min_x) / span_x * (size - 1)
    py = (1.0 - (y - min_y) / span_y) * (size - 1)
    return px, py


def draw_point_set(
    image: Image.Image,
    points: np.ndarray,
    view_name: str,
    bounds: tuple[float, float, float, float],
    color: tuple[int, int, int, int],
    radius: int,
) -> None:
    if points.size == 0:
        return
    x, y, depth = project_points(points, view_name)
    px, py = map_to_canvas(x, y, bounds, image.size[0])
    order = np.argsort(depth)
    overlay = Image.new("RGBA", image.size, (255, 255, 255, 0))
    draw = ImageDraw.Draw(overlay, "RGBA")
    for idx in order:
        cx = float(px[idx])
        cy = float(py[idx])
        draw.ellipse(
            [cx - radius, cy - radius, cx + radius, cy + radius],
            fill=color,
        )
    image.alpha_composite(overlay)


def draw_grid(draw: ImageDraw.ImageDraw, panel_width: int, panel_height: int) -> None:
    for ratio in (0.25, 0.5, 0.75):
        x = int(panel_width * ratio)
        y = int(panel_height * ratio)
        draw.line([(x, 0), (x, panel_height)], fill=GRID, width=1)
        draw.line([(0, y), (panel_width, y)], fill=GRID, width=1)


def render_projection_panel(
    point_sets: list[dict[str, Any]],
    reference_points: np.ndarray,
    view_name: str,
    panel_size: int,
    title: str,
) -> Image.Image:
    panel = Image.new("RGBA", (panel_size, panel_size + 28), WHITE)
    draw = ImageDraw.Draw(panel)
    font = choose_font(14)
    draw.text((10, 6), title, fill=TEXT, font=font)

    canvas = Image.new("RGBA", (panel_size, panel_size), WHITE)
    canvas_draw = ImageDraw.Draw(canvas)
    draw_grid(canvas_draw, panel_size, panel_size)
    bounds = compute_bounds(reference_points, view_name)
    for point_set in point_sets:
        draw_point_set(
            canvas,
            point_set["points"],
            view_name,
            bounds,
            point_set["color"],
            point_set["radius"],
        )
    panel.alpha_composite(canvas, (0, 28))
    return panel


def render_projection_preview(
    point_sets: list[dict[str, Any]],
    reference_points: np.ndarray,
    output_path: Path,
    title_prefix: str,
    panel_size: int,
) -> None:
    panels = []
    for view_name in VIEW_NAMES:
        panels.append(
            render_projection_panel(
                point_sets,
                reference_points,
                view_name,
                panel_size,
                f"{title_prefix} | {view_name}",
            )
        )
    preview = Image.new(
        "RGBA",
        (len(panels) * panel_size, panels[0].size[1]),
        WHITE,
    )
    for index, panel in enumerate(panels):
        preview.alpha_composite(panel, (index * panel_size, 0))
    preview.convert("RGB").save(output_path)


def render_open3d_preview(
    point_sets: list[dict[str, Any]],
    reference_points: np.ndarray,
    output_path: Path,
    title_prefix: str,
    panel_size: int,
) -> None:
    import open3d as o3d  # imported only when headless offscreen is known to work

    panels = []
    for view_name in VIEW_NAMES:
        renderer = o3d.visualization.rendering.OffscreenRenderer(panel_size, panel_size)
        renderer.scene.set_background([1.0, 1.0, 1.0, 1.0])
        renderer.scene.scene.enable_sun_light(False)
        renderer.scene.show_axes(False)

        for index, point_set in enumerate(point_sets):
            if point_set["points"].size == 0:
                continue
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(point_set["points"])
            rgb = np.asarray(point_set["color"][:3], dtype=np.float64) / 255.0
            colors = np.tile(rgb[None, :], (point_set["points"].shape[0], 1))
            cloud.colors = o3d.utility.Vector3dVector(colors)
            material = o3d.visualization.rendering.MaterialRecord()
            material.shader = "defaultUnlit"
            material.point_size = float(max(3, point_set["radius"] * 2 + 1))
            renderer.scene.add_geometry(f"points_{index}", cloud, material)

        if reference_points.size == 0:
            reference_points = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
        center = reference_points.mean(axis=0)
        extents = np.ptp(reference_points, axis=0)
        scale = max(float(np.max(extents)), 1.0)
        if view_name == "top":
            eye = center + np.array([0.0, 0.0, 2.8 * scale], dtype=np.float64)
            up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        elif view_name == "front":
            eye = center + np.array([0.0, -2.8 * scale, 0.0], dtype=np.float64)
            up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        else:
            eye = center + np.array([2.2, -2.2, 1.8], dtype=np.float64) * scale
            up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        renderer.setup_camera(60.0, center, eye, up)
        image = np.asarray(renderer.render_to_image())
        panel = Image.fromarray(image).convert("RGBA")
        header = Image.new("RGBA", (panel_size, panel_size + 28), WHITE)
        draw = ImageDraw.Draw(header)
        draw.text((10, 6), f"{title_prefix} | {view_name}", fill=TEXT, font=choose_font(14))
        header.alpha_composite(panel, (0, 28))
        panels.append(header)
        renderer.scene.clear_geometry()

    preview = Image.new("RGBA", (len(panels) * panel_size, panels[0].size[1]), WHITE)
    for index, panel in enumerate(panels):
        preview.alpha_composite(panel, (index * panel_size, 0))
    preview.convert("RGB").save(output_path)


def try_headless_open3d() -> tuple[bool, str]:
    check_code = textwrap.dedent(
        """
        import numpy as np
        import open3d as o3d
        pts = np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]], dtype=float)
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts)
        renderer = o3d.visualization.rendering.OffscreenRenderer(96, 96)
        renderer.scene.set_background([1.0, 1.0, 1.0, 1.0])
        mat = o3d.visualization.rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        mat.point_size = 6.0
        renderer.scene.add_geometry("cloud", cloud, mat)
        center = pts.mean(axis=0)
        renderer.setup_camera(60.0, center, center + np.array([1.8, -1.8, 1.4]), [0.0, 0.0, 1.0])
        renderer.render_to_image()
        print("ok")
        """
    ).strip()
    env = dict(os.environ)
    env.setdefault("MPLCONFIGDIR", "/tmp/mpl")
    completed = subprocess.run(
        [sys.executable, "-c", check_code],
        capture_output=True,
        text=True,
        env=env,
    )
    message = completed.stdout.strip() or completed.stderr.strip()
    message = message.replace("\n", " | ")
    return (completed.returncode == 0, message if message else f"returncode={completed.returncode}")


def render_preview(
    point_sets: list[dict[str, Any]],
    reference_points: np.ndarray,
    output_path: Path,
    title_prefix: str,
    panel_size: int,
    renderer_mode: str,
) -> None:
    if renderer_mode == "open3d_offscreen":
        render_open3d_preview(point_sets, reference_points, output_path, title_prefix, panel_size)
    else:
        render_projection_preview(point_sets, reference_points, output_path, title_prefix, panel_size)


def build_contact_sheet(
    preview_items: list[dict[str, Any]],
    image_field: str,
    output_path: Path,
    title: str,
    thumbnail_width: int,
    columns: int,
) -> None:
    font = choose_font(14)
    title_font = choose_font(18)
    margin = 18
    caption_height = 44
    if not preview_items:
        image = Image.new("RGBA", (900, 180), EMPTY_BG)
        draw = ImageDraw.Draw(image)
        draw.text((24, 24), title, fill=TEXT, font=title_font)
        draw.text((24, 72), "No voxels in this group.", fill=TEXT, font=font)
        image.convert("RGB").save(output_path)
        return

    thumbnails = []
    resampling = getattr(Image, "Resampling", Image)
    for item in preview_items:
        preview = Image.open(item[image_field]).convert("RGB")
        scale = thumbnail_width / preview.size[0]
        thumb_height = max(1, int(round(preview.size[1] * scale)))
        thumb = preview.resize((thumbnail_width, thumb_height), resampling.BILINEAR)
        thumbnails.append((item, thumb))

    tile_height = max(thumb.size[1] for _, thumb in thumbnails) + caption_height
    rows = int(np.ceil(len(thumbnails) / float(columns)))
    width = columns * thumbnail_width + (columns + 1) * margin
    height = rows * tile_height + (rows + 1) * margin + 32
    sheet = Image.new("RGBA", (width, height), WHITE)
    draw = ImageDraw.Draw(sheet)
    draw.text((margin, margin - 4), title, fill=TEXT, font=title_font)

    for index, (item, thumb) in enumerate(thumbnails):
        row = index // columns
        column = index % columns
        x = margin + column * thumbnail_width
        y = margin + 28 + row * tile_height
        sheet.paste(thumb, (x, y))
        caption_y = y + thumb.size[1] + 4
        caption = (
            f"{item['scene_name']} | {item['group_short']}\n"
            f"{item['voxel_id']} | score={float(item['relabel_score']):.3f}\n"
            f"baseline={item['baseline_rank'] or '-'} hard={item['hard_rank'] or '-'} scored={item['scored_rank'] or '-'}"
        )
        draw.multiline_text((x, caption_y), caption, fill=TEXT, font=font, spacing=2)
    sheet.convert("RGB").save(output_path)


def write_annotation_csv(output_path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "scene_name",
        "scene_path",
        "group",
        "voxel_id",
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
        "baseline_rank",
        "hard_rank",
        "scored_rank",
        "original_voxel_pcd",
        "original_neighborhood_pcd",
        "source_manifest_csv",
        "source_annotation_csv",
        "voxel_preview_png",
        "neighborhood_preview_png",
        "overlay_preview_png",
        "human_label",
        "confidence",
        "notes",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def write_decision_template(output_path: Path) -> None:
    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# Final Go / No-Go Decision Template\n\n")
        handle.write("## Decision\n\n")
        handle.write("- outcome: `continue` / `pivot` / `stop`\n")
        handle.write("- reviewer:\n")
        handle.write("- date:\n\n")
        handle.write("## Criteria\n\n")
        handle.write("- Are most `scored_only_relabeled` voxels visually plausible as `junction_like_mixed` rather than `planar_boundary`?\n")
        handle.write("- Do `hard_only_relabeled` voxels look less convincing than `scored_only_relabeled` voxels?\n")
        handle.write("- Is the gain from scored relabeling more than semantic reinterpretation of already-interesting planar voxels?\n\n")
        handle.write("## Scene-by-Scene Notes\n\n")
        handle.write("- `full0`:\n")
        handle.write("- `full153`:\n")
        handle.write("- `full95`:\n\n")
        handle.write("## Summary\n\n")
        handle.write("- strongest evidence:\n")
        handle.write("- weakest evidence:\n")
        handle.write("- next action if continue:\n")
        handle.write("- next action if pivot/stop:\n")


def write_review_index(
    output_path: Path,
    package_rows: list[dict[str, Any]],
    review_root: Path,
    renderer_mode: str,
    renderer_detail: str,
) -> None:
    group_counts = {
        group: sum(1 for row in package_rows if row["group"] == group)
        for group in GROUP_NAMES
    }
    scored_only_rows = [row for row in package_rows if row["group"] == "scored_only_relabeled"]
    hard_only_rows = [row for row in package_rows if row["group"] == "hard_only_relabeled"]

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# Final Manual Review Package\n\n")
        handle.write(f"- source review root: `{review_root}`\n")
        handle.write(f"- renderer mode: `{renderer_mode}`\n")
        handle.write(f"- renderer detail: `{renderer_detail}`\n")
        handle.write(f"- total reviewed voxels: {len(package_rows)}\n")
        handle.write(
            f"- group counts: scored_only={group_counts['scored_only_relabeled']}, "
            f"hard_only={group_counts['hard_only_relabeled']}, "
            f"shared={group_counts['shared_relabeled']}\n\n"
        )

        handle.write("## Open These First\n\n")
        handle.write("- `contact_sheets/scored_only_relabeled_contact_sheet.png`\n")
        handle.write("- `contact_sheets/hard_only_relabeled_contact_sheet.png`\n")
        handle.write("- `annotation_template.csv`\n")
        handle.write("- `decision_template.md`\n")
        handle.write("- `review_index.md`\n")
        handle.write("- source cross-scene summary: `source/cross_scene_summary.md`\n\n")

        handle.write("## Suggested Inspection Order\n\n")
        handle.write("1. Review `scored_only_relabeled` contact sheet first.\n")
        handle.write("2. Open scene-level `scored_only` contact sheets for `full0`, `full153`, then `full95`.\n")
        handle.write("3. Compare with `hard_only_relabeled` contact sheet to see whether scored-only looks more plausible.\n")
        handle.write("4. For uncertain cases, open the per-voxel `overlay_preview.png`, then `neighborhood_preview.png`, then the original PCD paths listed in `annotation_template.csv`.\n")
        handle.write("5. Fill `annotation_template.csv`, then complete `decision_template.md`.\n\n")

        handle.write("## Group Preview Locations\n\n")
        handle.write("- top-level scored-only contact sheet: `contact_sheets/scored_only_relabeled_contact_sheet.png`\n")
        handle.write("- top-level hard-only contact sheet: `contact_sheets/hard_only_relabeled_contact_sheet.png`\n")
        handle.write("- top-level shared contact sheet: `contact_sheets/shared_relabeled_contact_sheet.png`\n")
        handle.write("- scene-level previews live under `scenes/<scene>/groups/<group>/`\n\n")

        handle.write("## Exact Scored-Only Voxels\n\n")
        for row in scored_only_rows:
            handle.write(
                f"- `{row['scene_name']}` / `{row['voxel_id']}`: "
                f"`{row['group_dir_rel']}/voxels/{row['voxel_id']}/overlay_preview.png`\n"
            )

        handle.write("\n## Exact Hard-Only Voxels\n\n")
        if hard_only_rows:
            for row in hard_only_rows:
                handle.write(
                    f"- `{row['scene_name']}` / `{row['voxel_id']}`: "
                    f"`{row['group_dir_rel']}/voxels/{row['voxel_id']}/overlay_preview.png`\n"
                )
        else:
            handle.write("- (none)\n")

        handle.write("\n## Go / No-Go Review Instructions\n\n")
        handle.write("- Label each voxel using: `junction_like_mixed`, `planar_boundary`, `ambiguous`, `noise_or_bad_case`.\n")
        handle.write("- If most `scored_only` voxels look like clear `junction_like_mixed`, that supports continuing.\n")
        handle.write("- If `scored_only` mostly looks like ordinary planar boundary expansion, the gain is superficial and the method should pivot or stop.\n")
        handle.write("- Use `hard_only` as a negative comparison set when possible.\n")


def load_review_entries(review_root: Path) -> list[dict[str, Any]]:
    cross_scene_rows = read_csv_rows(review_root / "cross_scene_summary.csv")
    entries: list[dict[str, Any]] = []

    for scene_row in sorted(cross_scene_rows, key=lambda row: scene_sort_key(row["scene_name"])):
        scene_name = scene_row["scene_name"]
        scene_stem = Path(scene_name).stem
        scene_root = review_root / "scenes" / scene_stem
        review_rows = {
            row["voxel_id"]: row
            for row in read_csv_rows(scene_root / "relabeled_review_summary.csv")
        }
        for group in GROUP_NAMES:
            manifest_path = scene_root / "groups" / group / "manifest.csv"
            manifest_rows = read_csv_rows(manifest_path)
            for manifest_row in manifest_rows:
                voxel_id = manifest_row["voxel_id"]
                merged = dict(review_rows[voxel_id])
                merged["scene_name"] = scene_name
                merged["scene_path"] = scene_row["scene_path"]
                merged["group_short"] = group.replace("_relabeled", "")
                merged["group_dir_rel"] = str(Path("scenes") / scene_stem / "groups" / group)
                merged["source_manifest_rel"] = str((scene_root / "groups" / group / "manifest.csv").relative_to(review_root))
                merged["source_annotation_rel"] = str((scene_root / "groups" / group / "annotation_template.csv").relative_to(review_root))
                merged["source_scene_annotation_rel"] = str((scene_root / "annotation_template.csv").relative_to(review_root))
                merged["original_voxel_pcd_rel"] = str((scene_root / manifest_row["voxel_pcd_path"]).relative_to(review_root))
                merged["original_neighborhood_pcd_rel"] = str((scene_root / manifest_row["neighborhood_pcd_path"]).relative_to(review_root))
                merged["baseline_rank_value"] = safe_int(merged["baseline_rank"])
                merged["hard_rank_value"] = safe_int(merged["hard_rank"])
                merged["scored_rank_value"] = safe_int(merged["scored_rank"])
                entries.append(merged)
    entries.sort(
        key=lambda row: (
            scene_sort_key(row["scene_name"]),
            GROUP_NAMES.index(row["group"]),
            10_000 if row["scored_rank_value"] is None else row["scored_rank_value"],
            10_000 if row["hard_rank_value"] is None else row["hard_rank_value"],
            row["voxel_id"],
        )
    )
    return entries


def build_package_rows(
    repo_root: Path,
    review_root: Path,
    output_root: Path,
    review_entries: list[dict[str, Any]],
    renderer_mode: str,
    panel_size: int,
) -> list[dict[str, Any]]:
    package_rows: list[dict[str, Any]] = []
    group_colors = {
        "scored_only_relabeled": VOXEL_COLOR,
        "hard_only_relabeled": SHARED_COLOR,
        "shared_relabeled": SHARED_COLOR,
    }
    for entry in review_entries:
        scene_stem = Path(entry["scene_name"]).stem
        voxel_dir = output_root / "scenes" / scene_stem / "groups" / entry["group"] / "voxels" / entry["voxel_id"]
        voxel_dir.mkdir(parents=True, exist_ok=True)

        original_voxel_path = review_root / entry["original_voxel_pcd_rel"]
        original_neighborhood_path = review_root / entry["original_neighborhood_pcd_rel"]
        voxel_points = load_xyz_points(original_voxel_path)
        neighborhood_points = load_xyz_points(original_neighborhood_path)

        voxel_preview_path = voxel_dir / "voxel_preview.png"
        neighborhood_preview_path = voxel_dir / "neighborhood_preview.png"
        overlay_preview_path = voxel_dir / "overlay_preview.png"

        render_preview(
            [{"points": voxel_points, "color": group_colors[entry["group"]], "radius": 2}],
            voxel_points,
            voxel_preview_path,
            f"{entry['scene_name']} | {entry['voxel_id']} | voxel",
            panel_size,
            renderer_mode,
        )
        render_preview(
            [{"points": neighborhood_points, "color": NEIGHBOR_COLOR, "radius": 1}],
            neighborhood_points,
            neighborhood_preview_path,
            f"{entry['scene_name']} | {entry['voxel_id']} | neighborhood",
            panel_size,
            renderer_mode,
        )
        render_preview(
            [
                {"points": neighborhood_points, "color": NEIGHBOR_COLOR, "radius": 1},
                {"points": voxel_points, "color": group_colors[entry["group"]], "radius": 2},
            ],
            neighborhood_points,
            overlay_preview_path,
            f"{entry['scene_name']} | {entry['voxel_id']} | overlay",
            panel_size,
            renderer_mode,
        )

        package_row = {
            "scene_name": entry["scene_name"],
            "scene_path": entry["scene_path"],
            "group": entry["group"],
            "group_short": entry["group_short"],
            "voxel_id": entry["voxel_id"],
            "base_label": entry["base_label"],
            "final_label": entry["final_label"],
            "relabel_mode": entry["relabel_mode"],
            "relabel_score": entry["relabel_score"],
            "junction_score": entry["junction_score"],
            "junction_neighbor_count": entry["junction_neighbor_count"],
            "junction_cluster_count": entry["junction_cluster_count"],
            "junction_orientation_dispersion": entry["junction_orientation_dispersion"],
            "junction_dominant_fraction": entry["junction_dominant_fraction"],
            "occupancy_asymmetry": entry["occupancy_asymmetry"],
            "normal_variation": entry["normal_variation"],
            "opposite_face_pair_ratio": entry["opposite_face_pair_ratio"],
            "planar_context_penalty": entry["planar_context_penalty"],
            "corner_context_bonus": entry["corner_context_bonus"],
            "baseline_rank": entry["baseline_rank"],
            "hard_rank": entry["hard_rank"],
            "scored_rank": entry["scored_rank"],
            "source_manifest_csv": str((review_root / entry["source_manifest_rel"]).relative_to(repo_root)),
            "source_annotation_csv": str((review_root / entry["source_annotation_rel"]).relative_to(repo_root)),
            "original_voxel_pcd": str((review_root / entry["original_voxel_pcd_rel"]).relative_to(repo_root)),
            "original_neighborhood_pcd": str((review_root / entry["original_neighborhood_pcd_rel"]).relative_to(repo_root)),
            "voxel_preview_png": str(voxel_preview_path.relative_to(output_root)),
            "neighborhood_preview_png": str(neighborhood_preview_path.relative_to(output_root)),
            "overlay_preview_png": str(overlay_preview_path.relative_to(output_root)),
            "voxel_preview_abs": str(voxel_preview_path),
            "neighborhood_preview_abs": str(neighborhood_preview_path),
            "overlay_preview_abs": str(overlay_preview_path),
            "group_dir_rel": str((output_root / "scenes" / scene_stem / "groups" / entry["group"]).relative_to(output_root)),
            "human_label": "",
            "confidence": "",
            "notes": "",
        }
        package_rows.append(package_row)
    return package_rows


def copy_source_index(review_root: Path, output_root: Path) -> None:
    source_root = output_root / "source"
    source_root.mkdir(parents=True, exist_ok=True)
    for relative_path in (
        Path("cross_scene_summary.csv"),
        Path("cross_scene_summary.md"),
        Path("scene_selection.csv"),
        Path("scene_selection.md"),
    ):
        content = (review_root / relative_path).read_bytes()
        (source_root / relative_path.name).write_bytes(content)


def build_group_contact_sheets(output_root: Path, package_rows: list[dict[str, Any]], thumbnail_width: int, columns: int) -> None:
    top_contact_root = output_root / "contact_sheets"
    top_contact_root.mkdir(parents=True, exist_ok=True)
    for group in GROUP_NAMES:
        group_rows = [row for row in package_rows if row["group"] == group]
        build_contact_sheet(
            group_rows,
            "overlay_preview_abs",
            top_contact_root / f"{group}_contact_sheet.png",
            f"{group} overlay previews",
            thumbnail_width,
            columns,
        )

    scene_names = sorted({row["scene_name"] for row in package_rows}, key=scene_sort_key)
    for scene_name in scene_names:
        scene_stem = Path(scene_name).stem
        for group in GROUP_NAMES:
            group_rows = [row for row in package_rows if row["scene_name"] == scene_name and row["group"] == group]
            group_dir = output_root / "scenes" / scene_stem / "groups" / group
            group_dir.mkdir(parents=True, exist_ok=True)
            build_contact_sheet(
                group_rows,
                "overlay_preview_abs",
                group_dir / "contact_sheet.png",
                f"{scene_name} | {group}",
                thumbnail_width,
                columns,
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="Build deterministic PNG review package from multiscene relabeled voxel exports.")
    parser.add_argument("--review-root", default="results/context_refinement_scored_multiscene_review")
    parser.add_argument("--output-root", default="results/final_manual_review_package")
    parser.add_argument("--panel-size", type=int, default=320)
    parser.add_argument("--thumbnail-width", type=int, default=360)
    parser.add_argument("--contact-columns", type=int, default=2)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    review_root = (repo_root / args.review_root).resolve() if not Path(args.review_root).is_absolute() else Path(args.review_root)
    output_root = (repo_root / args.output_root).resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    headless_ok, renderer_detail = try_headless_open3d()
    renderer_mode = "open3d_offscreen" if headless_ok else "projection_fallback"

    review_entries = load_review_entries(review_root)
    copy_source_index(review_root, output_root)
    package_rows = build_package_rows(
        repo_root,
        review_root,
        output_root,
        review_entries,
        renderer_mode,
        args.panel_size,
    )

    build_group_contact_sheets(output_root, package_rows, args.thumbnail_width, args.contact_columns)
    write_annotation_csv(output_root / "annotation_template.csv", package_rows)
    write_decision_template(output_root / "decision_template.md")
    write_review_index(output_root / "review_index.md", package_rows, review_root, renderer_mode, renderer_detail)

    print(f"Wrote {output_root / 'review_index.md'}")
    print(f"Wrote {output_root / 'annotation_template.csv'}")
    print(f"Wrote {output_root / 'decision_template.md'}")
    print(f"Renderer mode: {renderer_mode} ({renderer_detail})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
