# Scene Review Summary: full153.pcd

- scene path: `/home/chani/personal/CVRL_SLAM_Docker/GES-Voxel-Docker/prev/BALM/datas/benchmark_realworld/full153.pcd`
- point count: 71823
- selection reason: `point_count_25pct`

## Top-k Comparison

- baseline `context_hybrid`: combined=12, planar=143
- hard-threshold `junction_mixed_priority`: combined=14, planar=138
- scored `junction_mixed_scored` (threshold=0.66): combined=18, planar=134

## Group Counts

- `shared_relabeled`: count=3, baseline_topk=2, hard_topk=2, scored_topk=3
- `scored_only_relabeled`: count=7, baseline_topk=3, hard_topk=3, scored_topk=3
- `hard_only_relabeled`: count=0, baseline_topk=0, hard_topk=0, scored_topk=0

## Exact Voxel IDs

- `shared_relabeled`: -3_9_-1, -6_-1_5, -3_9_0
- `scored_only_relabeled`: 6_2_2, 1_6_4, -1_9_-1, -1_8_-2, 2_5_6, 2_6_4, 7_1_2
- `hard_only_relabeled`: (none)

## Files To Inspect First

- scored-only combined neighborhood: `groups/scored_only_relabeled/combined_neighborhood_radius2.pcd`
- scored-only annotation template: `groups/scored_only_relabeled/annotation_template.csv`
- hard-only combined neighborhood: `groups/hard_only_relabeled/combined_neighborhood_radius2.pcd`
- hard-only annotation template: `groups/hard_only_relabeled/annotation_template.csv`
- combined annotation template: `annotation_template.csv`
