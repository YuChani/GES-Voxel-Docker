# Final Manual Review Package

- source review root: `/home/chani/personal/CVRL_SLAM_Docker/GES-Voxel-Docker/results/context_refinement_scored_multiscene_review`
- renderer mode: `projection_fallback`
- renderer detail: `eglInitialize failed`
- total reviewed voxels: 29
- group counts: scored_only=16, hard_only=2, shared=11

## Open These First

- `contact_sheets/scored_only_relabeled_contact_sheet.png`
- `contact_sheets/hard_only_relabeled_contact_sheet.png`
- `annotation_template.csv`
- `decision_template.md`
- `review_index.md`
- source cross-scene summary: `source/cross_scene_summary.md`

## Suggested Inspection Order

1. Review `scored_only_relabeled` contact sheet first.
2. Open scene-level `scored_only` contact sheets for `full0`, `full153`, then `full95`.
3. Compare with `hard_only_relabeled` contact sheet to see whether scored-only looks more plausible.
4. For uncertain cases, open the per-voxel `overlay_preview.png`, then `neighborhood_preview.png`, then the original PCD paths listed in `annotation_template.csv`.
5. Fill `annotation_template.csv`, then complete `decision_template.md`.

## Group Preview Locations

- top-level scored-only contact sheet: `contact_sheets/scored_only_relabeled_contact_sheet.png`
- top-level hard-only contact sheet: `contact_sheets/hard_only_relabeled_contact_sheet.png`
- top-level shared contact sheet: `contact_sheets/shared_relabeled_contact_sheet.png`
- scene-level previews live under `scenes/<scene>/groups/<group>/`

## Exact Scored-Only Voxels

- `full0.pcd` / `0_-10_6`: `scenes/full0/groups/scored_only_relabeled/voxels/0_-10_6/overlay_preview.png`
- `full0.pcd` / `5_4_4`: `scenes/full0/groups/scored_only_relabeled/voxels/5_4_4/overlay_preview.png`
- `full0.pcd` / `-2_-8_4`: `scenes/full0/groups/scored_only_relabeled/voxels/-2_-8_4/overlay_preview.png`
- `full0.pcd` / `6_7_-2`: `scenes/full0/groups/scored_only_relabeled/voxels/6_7_-2/overlay_preview.png`
- `full0.pcd` / `1_-10_4`: `scenes/full0/groups/scored_only_relabeled/voxels/1_-10_4/overlay_preview.png`
- `full95.pcd` / `-9_-6_2`: `scenes/full95/groups/scored_only_relabeled/voxels/-9_-6_2/overlay_preview.png`
- `full95.pcd` / `-8_0_5`: `scenes/full95/groups/scored_only_relabeled/voxels/-8_0_5/overlay_preview.png`
- `full95.pcd` / `-8_3_0`: `scenes/full95/groups/scored_only_relabeled/voxels/-8_3_0/overlay_preview.png`
- `full95.pcd` / `-8_4_0`: `scenes/full95/groups/scored_only_relabeled/voxels/-8_4_0/overlay_preview.png`
- `full153.pcd` / `6_2_2`: `scenes/full153/groups/scored_only_relabeled/voxels/6_2_2/overlay_preview.png`
- `full153.pcd` / `1_6_4`: `scenes/full153/groups/scored_only_relabeled/voxels/1_6_4/overlay_preview.png`
- `full153.pcd` / `-1_9_-1`: `scenes/full153/groups/scored_only_relabeled/voxels/-1_9_-1/overlay_preview.png`
- `full153.pcd` / `-1_8_-2`: `scenes/full153/groups/scored_only_relabeled/voxels/-1_8_-2/overlay_preview.png`
- `full153.pcd` / `2_5_6`: `scenes/full153/groups/scored_only_relabeled/voxels/2_5_6/overlay_preview.png`
- `full153.pcd` / `2_6_4`: `scenes/full153/groups/scored_only_relabeled/voxels/2_6_4/overlay_preview.png`
- `full153.pcd` / `7_1_2`: `scenes/full153/groups/scored_only_relabeled/voxels/7_1_2/overlay_preview.png`

## Exact Hard-Only Voxels

- `full0.pcd` / `6_12_0`: `scenes/full0/groups/hard_only_relabeled/voxels/6_12_0/overlay_preview.png`
- `full0.pcd` / `6_12_1`: `scenes/full0/groups/hard_only_relabeled/voxels/6_12_1/overlay_preview.png`

## Go / No-Go Review Instructions

- Label each voxel using: `junction_like_mixed`, `planar_boundary`, `ambiguous`, `noise_or_bad_case`.
- If most `scored_only` voxels look like clear `junction_like_mixed`, that supports continuing.
- If `scored_only` mostly looks like ordinary planar boundary expansion, the gain is superficial and the method should pivot or stop.
- Use `hard_only` as a negative comparison set when possible.
