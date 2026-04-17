# Junction Mixed Scored Calibration Summary

## Baseline and Hard-Threshold

- `context_hybrid` (`hard_threshold`, threshold=n/a): combined=32, planar=126, relabeled=0, top-k relabeled=0
- `junction_mixed_priority` (`hard_threshold`, threshold=n/a): combined=35, planar=123, relabeled=10, top-k relabeled=3
- `junction_mixed_scored` (`scored`, threshold=0.66): combined=40, planar=119, relabeled=13, top-k relabeled=8

## Scored Threshold Sweep

- threshold `0.58`: combined=40 (+8), planar=119 (-7), relabeled=18, top-k relabeled=8
- threshold `0.62`: combined=40 (+8), planar=119 (-7), relabeled=17, top-k relabeled=8
- threshold `0.66`: combined=40 (+8), planar=119 (-7), relabeled=13, top-k relabeled=8
- threshold `0.70`: combined=34 (+2), planar=124 (-2), relabeled=7, top-k relabeled=2
- threshold `0.74`: combined=34 (+2), planar=124 (-2), relabeled=6, top-k relabeled=2

## Relabel Score Aggregate Check

- `average_junction_mixed_relabel_score` and `average_junction_mixed_relabel_score_all_voxels` are expected to be identical across modes because the relabel score is computed once per analyzed voxel before thresholding.
- Mode-sensitive aggregates are `average_relabeled_junction_mixed_relabel_score` and `average_topk_relabeled_junction_mixed_relabel_score`.
- Current scored threshold `0.66`: all_voxels=0.2461, planar=0.1964, relabeled=0.7283, top-k relabeled=0.7023
