# Scene Review Summary: full0.pcd

- scene path: `/home/chani/personal/CVRL_SLAM_Docker/GES-Voxel-Docker/prev/BALM/datas/benchmark_realworld/full0.pcd`
- point count: 73452
- selection reason: `anchor_full0`

## Top-k Comparison

- baseline `context_hybrid`: combined=32, planar=126
- hard-threshold `junction_mixed_priority`: combined=35, planar=123
- scored `junction_mixed_scored` (threshold=0.66): combined=40, planar=119

## Group Counts

- `shared_relabeled`: count=8, baseline_topk=3, hard_topk=3, scored_topk=3
- `scored_only_relabeled`: count=5, baseline_topk=2, hard_topk=4, scored_topk=5
- `hard_only_relabeled`: count=2, baseline_topk=0, hard_topk=0, scored_topk=0

## Exact Voxel IDs

- `shared_relabeled`: 6_5_-1, 4_-13_4, 5_-13_4, 5_12_-1, 5_13_-1, 6_13_-1, 6_13_1, 6_5_-2
- `scored_only_relabeled`: 0_-10_6, 5_4_4, -2_-8_4, 6_7_-2, 1_-10_4
- `hard_only_relabeled`: 6_12_0, 6_12_1

## Files To Inspect First

- scored-only combined neighborhood: `groups/scored_only_relabeled/combined_neighborhood_radius2.pcd`
- scored-only annotation template: `groups/scored_only_relabeled/annotation_template.csv`
- hard-only combined neighborhood: `groups/hard_only_relabeled/combined_neighborhood_radius2.pcd`
- hard-only annotation template: `groups/hard_only_relabeled/annotation_template.csv`
- combined annotation template: `annotation_template.csv`
