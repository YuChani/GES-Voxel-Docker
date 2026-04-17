# Junction Mixed Scored Reclassification Summary

## Top-k Comparison

- `context_hybrid` (`hard_threshold`): corner_like=32, junction_like_mixed=0, combined=32, planar=126, topk_relabeled=0
- `junction_mixed_priority` (`hard_threshold`): corner_like=32, junction_like_mixed=3, combined=35, planar=123, topk_relabeled=3
- `junction_mixed_scored` (`scored`): corner_like=32, junction_like_mixed=8, combined=40, planar=119, topk_relabeled=8

- `junction_mixed_priority` combined delta vs baseline: +3, planar delta vs baseline: -3
- `junction_mixed_scored` combined delta vs baseline: +8, planar delta vs baseline: -7

## Global Relabel Counts

- `context_hybrid`: junction_like_mixed=0, combined=33, planar_reduction=0, average_relabel_score=0.2461
- `junction_mixed_priority`: junction_like_mixed=10, combined=43, planar_reduction=10, average_relabel_score=0.2461
- `junction_mixed_scored`: junction_like_mixed=13, combined=46, planar_reduction=13, average_relabel_score=0.2461

## Parameters

- `junction_mixed_relabel_mode` = `scored`
- `junction_mixed_min_neighbor_count` = `10`
- `junction_mixed_min_cluster_count` = `3`
- `junction_mixed_min_score` = `0.72`
- `junction_mixed_min_orientation_dispersion` = `0.48`
- `junction_mixed_max_dominant_fraction` = `0.38`
- `junction_mixed_min_occupancy_asymmetry` = `0.3`
- `junction_mixed_min_normal_variation` = `0.2`
- `junction_mixed_max_opposite_face_pair_ratio` = `0.67`
- `junction_mixed_scored_min_neighbor_count` = `8`
- `junction_mixed_scored_min_cluster_count` = `2`
- `junction_mixed_scored_min_junction_score` = `0.3`
- `junction_mixed_scored_min_orientation_dispersion` = `0.35`
- `junction_mixed_scored_max_dominant_fraction` = `0.62`
- `junction_mixed_scored_min_occupancy_asymmetry` = `0.18`
- `junction_mixed_scored_min_normal_variation` = `0.1`
- `junction_mixed_scored_threshold` = `0.66`
