# Junction-Like Mixed Review Export Summary

## Group Counts

- `shared_relabeled`: count=0, baseline_topk=0, hard_topk=0, scored_topk=0
- `scored_only_relabeled`: count=4, baseline_topk=1, hard_topk=1, scored_topk=1
- `hard_only_relabeled`: count=0, baseline_topk=0, hard_topk=0, scored_topk=0

## Group Feature Means

- `shared_relabeled`: relabel_score=0.0000, junction_score=0.0000, cluster_count=0.00, dispersion=0.0000, dominant_fraction=0.0000, occupancy_asymmetry=0.0000, normal_variation=0.0000
- `scored_only_relabeled`: relabel_score=0.7322, junction_score=0.5898, cluster_count=2.75, dispersion=0.6195, dominant_fraction=0.3946, occupancy_asymmetry=0.8846, normal_variation=0.2866
- `hard_only_relabeled`: relabel_score=0.0000, junction_score=0.0000, cluster_count=0.00, dispersion=0.0000, dominant_fraction=0.0000, occupancy_asymmetry=0.0000, normal_variation=0.0000

## Exact Voxel IDs

- `scored_only_relabeled`: -9_-6_2, -8_0_5, -8_3_0, -8_4_0
- `hard_only_relabeled`: (none)

## Files To Open First

- scored-only combined neighborhood: `groups/scored_only_relabeled/combined_neighborhood_radius2.pcd`
- hard-only combined neighborhood: `groups/hard_only_relabeled/combined_neighborhood_radius2.pcd`
- full review table: `relabeled_review_summary.csv`
- scored-only top-k voxel neighborhoods to inspect first:
  - voxel=-9_-6_2, scored_rank=1, `groups/scored_only_relabeled/voxels/-9_-6_2/neighborhood_radius2.pcd`

## Manifests

- `shared_relabeled` manifest: `groups/shared_relabeled/manifest.csv`
- `scored_only_relabeled` manifest: `groups/scored_only_relabeled/manifest.csv`
- `hard_only_relabeled` manifest: `groups/hard_only_relabeled/manifest.csv`
