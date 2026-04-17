# Junction-Like Mixed Review Export Summary

## Group Counts

- `shared_relabeled`: count=8, baseline_topk=3, hard_topk=3, scored_topk=3
- `scored_only_relabeled`: count=5, baseline_topk=2, hard_topk=4, scored_topk=5
- `hard_only_relabeled`: count=2, baseline_topk=0, hard_topk=0, scored_topk=0

## Group Feature Means

- `shared_relabeled`: relabel_score=0.7540, junction_score=0.7471, cluster_count=3.12, dispersion=0.5517, dominant_fraction=0.3221, occupancy_asymmetry=0.6379, normal_variation=0.2232
- `scored_only_relabeled`: relabel_score=0.6874, junction_score=0.7359, cluster_count=3.40, dispersion=0.4800, dominant_fraction=0.3429, occupancy_asymmetry=0.3864, normal_variation=0.2099
- `hard_only_relabeled`: relabel_score=0.6520, junction_score=0.7228, cluster_count=3.00, dispersion=0.5721, dominant_fraction=0.3038, occupancy_asymmetry=0.2804, normal_variation=0.2853

## Exact Voxel IDs

- `scored_only_relabeled`: 0_-10_6, 5_4_4, -2_-8_4, 6_7_-2, 1_-10_4
- `hard_only_relabeled`: 6_12_0, 6_12_1

## Files To Open First

- scored-only combined neighborhood: `groups/scored_only_relabeled/combined_neighborhood_radius2.pcd`
- hard-only combined neighborhood: `groups/hard_only_relabeled/combined_neighborhood_radius2.pcd`
- full review table: `relabeled_review_summary.csv`
- scored-only top-k voxel neighborhoods to inspect first:
  - voxel=0_-10_6, scored_rank=9, `groups/scored_only_relabeled/voxels/0_-10_6/neighborhood_radius2.pcd`
  - voxel=5_4_4, scored_rank=26, `groups/scored_only_relabeled/voxels/5_4_4/neighborhood_radius2.pcd`
  - voxel=-2_-8_4, scored_rank=60, `groups/scored_only_relabeled/voxels/-2_-8_4/neighborhood_radius2.pcd`
  - voxel=6_7_-2, scored_rank=88, `groups/scored_only_relabeled/voxels/6_7_-2/neighborhood_radius2.pcd`
  - voxel=1_-10_4, scored_rank=135, `groups/scored_only_relabeled/voxels/1_-10_4/neighborhood_radius2.pcd`
- hard-only voxel neighborhoods:
  - voxel=6_12_0, hard_rank=out_of_top_k, `groups/hard_only_relabeled/voxels/6_12_0/neighborhood_radius2.pcd`
  - voxel=6_12_1, hard_rank=out_of_top_k, `groups/hard_only_relabeled/voxels/6_12_1/neighborhood_radius2.pcd`

## Manifests

- `shared_relabeled` manifest: `groups/shared_relabeled/manifest.csv`
- `scored_only_relabeled` manifest: `groups/scored_only_relabeled/manifest.csv`
- `hard_only_relabeled` manifest: `groups/hard_only_relabeled/manifest.csv`
