# Junction-Like Mixed Review Export Summary

## Group Counts

- `shared_relabeled`: count=3, baseline_topk=2, hard_topk=2, scored_topk=3
- `scored_only_relabeled`: count=7, baseline_topk=3, hard_topk=3, scored_topk=3
- `hard_only_relabeled`: count=0, baseline_topk=0, hard_topk=0, scored_topk=0

## Group Feature Means

- `shared_relabeled`: relabel_score=0.8821, junction_score=0.7738, cluster_count=3.33, dispersion=0.5795, dominant_fraction=0.2782, occupancy_asymmetry=0.4738, normal_variation=0.4618
- `scored_only_relabeled`: relabel_score=0.7584, junction_score=0.6836, cluster_count=3.00, dispersion=0.4784, dominant_fraction=0.3569, occupancy_asymmetry=0.8080, normal_variation=0.4226
- `hard_only_relabeled`: relabel_score=0.0000, junction_score=0.0000, cluster_count=0.00, dispersion=0.0000, dominant_fraction=0.0000, occupancy_asymmetry=0.0000, normal_variation=0.0000

## Exact Voxel IDs

- `scored_only_relabeled`: 6_2_2, 1_6_4, -1_9_-1, -1_8_-2, 2_5_6, 2_6_4, 7_1_2
- `hard_only_relabeled`: (none)

## Files To Open First

- scored-only combined neighborhood: `groups/scored_only_relabeled/combined_neighborhood_radius2.pcd`
- hard-only combined neighborhood: `groups/hard_only_relabeled/combined_neighborhood_radius2.pcd`
- full review table: `relabeled_review_summary.csv`
- scored-only top-k voxel neighborhoods to inspect first:
  - voxel=6_2_2, scored_rank=1, `groups/scored_only_relabeled/voxels/6_2_2/neighborhood_radius2.pcd`
  - voxel=1_6_4, scored_rank=4, `groups/scored_only_relabeled/voxels/1_6_4/neighborhood_radius2.pcd`
  - voxel=-1_9_-1, scored_rank=6, `groups/scored_only_relabeled/voxels/-1_9_-1/neighborhood_radius2.pcd`

## Manifests

- `shared_relabeled` manifest: `groups/shared_relabeled/manifest.csv`
- `scored_only_relabeled` manifest: `groups/scored_only_relabeled/manifest.csv`
- `hard_only_relabeled` manifest: `groups/hard_only_relabeled/manifest.csv`
