# Scene Review Summary: full95.pcd

- scene path: `/home/chani/personal/CVRL_SLAM_Docker/GES-Voxel-Docker/prev/BALM/datas/benchmark_realworld/full95.pcd`
- point count: 79937
- selection reason: `point_count_75pct`

## Top-k Comparison

- baseline `context_hybrid`: combined=14, planar=163
- hard-threshold `junction_mixed_priority`: combined=15, planar=163
- scored `junction_mixed_scored` (threshold=0.66): combined=16, planar=162

## Group Counts

- `shared_relabeled`: count=0, baseline_topk=0, hard_topk=0, scored_topk=0
- `scored_only_relabeled`: count=4, baseline_topk=1, hard_topk=1, scored_topk=1
- `hard_only_relabeled`: count=0, baseline_topk=0, hard_topk=0, scored_topk=0

## Exact Voxel IDs

- `shared_relabeled`: (none)
- `scored_only_relabeled`: -9_-6_2, -8_0_5, -8_3_0, -8_4_0
- `hard_only_relabeled`: (none)

## Files To Inspect First

- scored-only combined neighborhood: `groups/scored_only_relabeled/combined_neighborhood_radius2.pcd`
- scored-only annotation template: `groups/scored_only_relabeled/annotation_template.csv`
- hard-only combined neighborhood: `groups/hard_only_relabeled/combined_neighborhood_radius2.pcd`
- hard-only annotation template: `groups/hard_only_relabeled/annotation_template.csv`
- combined annotation template: `annotation_template.csv`
