# Context Refinement Summary

## Baseline

- `nonplanar_priority`: corner_like=32/200 (0.160), planar=131/200 (0.655)

## Mode Comparison

- `context_hybrid`: corner_like=32/200 (0.160), planar=126/200 (0.630), delta_corner=+0.000, delta_planar=-0.025
- `context_face_support`: corner_like=32/200 (0.160), planar=127/200 (0.635), delta_corner=+0.000, delta_planar=-0.020
- `context_normal_variation`: corner_like=32/200 (0.160), planar=129/200 (0.645), delta_corner=+0.000, delta_planar=-0.010
- `context_asymmetry`: corner_like=32/200 (0.160), planar=130/200 (0.650), delta_corner=+0.000, delta_planar=-0.005
- `nonplanar_priority`: corner_like=32/200 (0.160), planar=131/200 (0.655), delta_corner=+0.000, delta_planar=+0.000

## Best Feature Ablation

- `context_face_support` reduced planar fraction by 0.020p and changed corner_like fraction by +0.000p versus baseline.

## Hybrid

- `context_hybrid`: corner_like=32/200 (0.160), planar=126/200 (0.630)
