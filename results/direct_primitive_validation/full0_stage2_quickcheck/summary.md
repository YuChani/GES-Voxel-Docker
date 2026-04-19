# Direct Primitive Validation Stage-2 Quickcheck

- input_path: `./prev/BALM/datas/benchmark_realworld/full0.pcd`
- output_dir: `/home/chani/personal/CVRL_SLAM_Docker/GES-Voxel-Docker/results/direct_primitive_validation/full0_stage2_quickcheck`
- input_points: `73452`
- loaded_files: `1`
- voxel_size: `1`
- min_points_per_voxel: `15`
- max_voxels: `12`
- per_category_limit: `4`
- selection_mode: `auto_small`
- registration_neighborhood_voxels: `1`
- registration_translation_step_ratio: `0.1`
- registration_rotation_degrees: `5`
- gaussian_model: `mean/covariance baseline`
- surface_model: `PCA local frame + quantile axis scales + L_p shell scaffold`
- note: boundary/mixed category는 semantic relabel이 아니라 `linear` voxel proxy를 사용한다.

## Normalization Definition

- `characteristic_spread_scale = mean(sqrt(eigenvalues))`
- `shell_geometric_average_residual = shell_average_residual * mean(shell_axis_scales)`
- `gaussian_normalized_residual_by_spread = average_center_distance / characteristic_spread_scale`
- `shell_normalized_residual_by_spread = shell_geometric_average_residual / characteristic_spread_scale`
- shell 쪽 geometric residual은 exact point-to-surface distance가 아니라 `|r-1| * mean(axis_scales)` 기반 근사다.

## Normalized Comparison Means

| selection_tag | count | gaussian_by_voxel | shell_by_voxel | gaussian_by_spread | shell_by_spread | gaussian_minus_shell |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| boundary_candidate_proxy | 4 | 0.231105 | 0.056378 | 2.506163 | 0.665616 | 1.840547 |
| corner_candidate | 4 | 0.384247 | 0.241818 | 1.678446 | 1.056747 | 0.621699 |
| planar_candidate | 4 | 0.377627 | 0.126071 | 1.953307 | 0.650145 | 1.303161 |

## Registration Quickcheck Means

| selection_tag | perturbation | cases | gaussian_delta | shell_delta | shell_minus_gaussian | shell_better_cases |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| boundary_candidate_proxy | rotate_middle | 4 | 0.055336 | 0.187510 | 0.132174 | 3 |
| boundary_candidate_proxy | translate_major | 4 | 0.001441 | 0.011714 | 0.010273 | 3 |
| boundary_candidate_proxy | translate_normal | 4 | 0.164708 | 0.472450 | 0.307742 | 2 |
| corner_candidate | rotate_middle | 4 | 0.012227 | 0.030091 | 0.017864 | 4 |
| corner_candidate | translate_major | 4 | 0.000093 | 0.003470 | 0.003376 | 2 |
| corner_candidate | translate_normal | 4 | 0.022712 | 0.050027 | 0.027315 | 3 |
| planar_candidate | rotate_middle | 4 | 0.292194 | 0.483445 | 0.191251 | 4 |
| planar_candidate | translate_major | 4 | 0.003714 | 0.003112 | -0.000602 | 1 |
| planar_candidate | translate_normal | 4 | 0.585719 | 0.823344 | 0.237625 | 3 |

## Selected Cases

- `-4_-10_6` | boundary_candidate_proxy | base_label=`linear` | points=`31` | neighborhood_points=`157` | gaussian_by_spread=`2.480964` | shell_by_spread=`0.677319` | gap=`1.803644`
- `2_15_-2` | boundary_candidate_proxy | base_label=`linear` | points=`21` | neighborhood_points=`253` | gaussian_by_spread=`2.477980` | shell_by_spread=`0.654227` | gap=`1.823754`
- `2_-9_4` | boundary_candidate_proxy | base_label=`linear` | points=`20` | neighborhood_points=`423` | gaussian_by_spread=`2.682111` | shell_by_spread=`0.368745` | gap=`2.313366`
- `1_-9_4` | boundary_candidate_proxy | base_label=`linear` | points=`16` | neighborhood_points=`625` | gaussian_by_spread=`2.383596` | shell_by_spread=`0.962174` | gap=`1.421422`
- `6_0_2` | corner_candidate | base_label=`corner_like` | points=`147` | neighborhood_points=`2188` | gaussian_by_spread=`1.703379` | shell_by_spread=`1.046346` | gap=`0.657033`
- `6_-1_2` | corner_candidate | base_label=`corner_like` | points=`141` | neighborhood_points=`2215` | gaussian_by_spread=`1.713154` | shell_by_spread=`0.994494` | gap=`0.718660`
- `3_-5_4` | corner_candidate | base_label=`corner_like` | points=`96` | neighborhood_points=`687` | gaussian_by_spread=`1.659343` | shell_by_spread=`1.124568` | gap=`0.534775`
- `6_14_1` | corner_candidate | base_label=`corner_like` | points=`26` | neighborhood_points=`215` | gaussian_by_spread=`1.637908` | shell_by_spread=`1.061580` | gap=`0.576328`
- `1_-2_-2` | planar_candidate | base_label=`planar` | points=`1447` | neighborhood_points=`8480` | gaussian_by_spread=`1.959498` | shell_by_spread=`0.545830` | gap=`1.413668`
- `0_2_-2` | planar_candidate | base_label=`planar` | points=`917` | neighborhood_points=`7041` | gaussian_by_spread=`1.964427` | shell_by_spread=`0.618594` | gap=`1.345833`
- `-2_-3_-2` | planar_candidate | base_label=`planar` | points=`644` | neighborhood_points=`5150` | gaussian_by_spread=`1.965166` | shell_by_spread=`0.620142` | gap=`1.345024`
- `6_2_3` | planar_candidate | base_label=`planar` | points=`170` | neighborhood_points=`1346` | gaussian_by_spread=`1.924136` | shell_by_spread=`0.816015` | gap=`1.108121`

## Interpretation Guardrails

- 이 quickcheck는 exact GES/GND fitting이 아니라 surface-shell proxy와 Gaussian baseline의 초기 분별력 검사용이다.
- registration quickcheck는 local neighborhood를 reference/source로 deterministic split한 뒤 nominal/perturbed score 변화를 보는 소규모 offline benchmark다.
- quickcheck score는 model-internal normalized residual이므로, full registration accuracy나 SLAM 성능을 아직 주장하지 않는다.
- shell 모델이 promising하다고 보려면 normalized residual에서 category별 이득이 반복되고, perturbation delta에서도 shell 쪽 분별력이 일관되게 커야 한다.
