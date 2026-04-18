# Direct Primitive Validation Stage-1

- input_path: `./prev/BALM/datas/benchmark_realworld/full0.pcd`
- output_dir: `/home/chani/personal/CVRL_SLAM_Docker/GES-Voxel-Docker/results/direct_primitive_validation/full0_stage1`
- input_points: `73452`
- loaded_files: `1`
- voxel_size: `1`
- min_points_per_voxel: `15`
- max_voxels: `12`
- per_category_limit: `4`
- selection_mode: `auto_small`
- gaussian_model: `mean/covariance baseline`
- surface_model: `PCA local frame + quantile axis scales + L_p shell scaffold`
- note: Stage-1의 boundary/mixed category는 robust semantic label이 아니라 `linear` voxel proxy를 사용한다.

## Category Means

| selection_tag | count | gaussian_avg_mahalanobis2 | gaussian_avg_center_distance | surface_normal_rms | shell_average_residual |
| --- | ---: | ---: | ---: | ---: | ---: |
| boundary_candidate_proxy | 4 | 0.968860 | 0.231105 | 0.000934 | 0.361987 |
| corner_candidate | 4 | 2.876136 | 0.384247 | 0.140919 | 0.676737 |
| planar_candidate | 4 | 2.065916 | 0.377627 | 0.009273 | 0.391939 |

## Selected Voxels

- `-4_-10_6` | boundary_candidate_proxy | base_label=`linear` | points=`31` | gaussian_avg_mahalanobis2=`1.012098` | shell_average_residual=`0.378249` | surface_normal_rms=`0.001824`
- `2_15_-2` | boundary_candidate_proxy | base_label=`linear` | points=`21` | gaussian_avg_mahalanobis2=`0.989594` | shell_average_residual=`0.376133` | surface_normal_rms=`0.000197`
- `2_-9_4` | boundary_candidate_proxy | base_label=`linear` | points=`20` | gaussian_avg_mahalanobis2=`0.971028` | shell_average_residual=`0.236245` | surface_normal_rms=`0.001286`
- `1_-9_4` | boundary_candidate_proxy | base_label=`linear` | points=`16` | gaussian_avg_mahalanobis2=`0.902719` | shell_average_residual=`0.457319` | surface_normal_rms=`0.000431`
- `6_0_2` | corner_candidate | base_label=`corner_like` | points=`147` | gaussian_avg_mahalanobis2=`2.906883` | shell_average_residual=`0.650923` | surface_normal_rms=`0.144141`
- `6_-1_2` | corner_candidate | base_label=`corner_like` | points=`141` | gaussian_avg_mahalanobis2=`2.896618` | shell_average_residual=`0.625733` | surface_normal_rms=`0.131796`
- `3_-5_4` | corner_candidate | base_label=`corner_like` | points=`96` | gaussian_avg_mahalanobis2=`2.870345` | shell_average_residual=`0.674317` | surface_normal_rms=`0.117136`
- `6_14_1` | corner_candidate | base_label=`corner_like` | points=`26` | gaussian_avg_mahalanobis2=`2.830697` | shell_average_residual=`0.755975` | surface_normal_rms=`0.170602`
- `1_-2_-2` | planar_candidate | base_label=`planar` | points=`1447` | gaussian_avg_mahalanobis2=`2.008804` | shell_average_residual=`0.310084` | surface_normal_rms=`0.006028`
- `0_2_-2` | planar_candidate | base_label=`planar` | points=`917` | gaussian_avg_mahalanobis2=`2.006504` | shell_average_residual=`0.370210` | surface_normal_rms=`0.005841`
- `-2_-3_-2` | planar_candidate | base_label=`planar` | points=`644` | gaussian_avg_mahalanobis2=`2.000140` | shell_average_residual=`0.370816` | surface_normal_rms=`0.005307`
- `6_2_3` | planar_candidate | base_label=`planar` | points=`170` | gaussian_avg_mahalanobis2=`2.248217` | shell_average_residual=`0.516646` | surface_normal_rms=`0.019915`

## Interpretation Guardrails

- 이 출력은 Stage-1 scaffold다. exact GES/GND fitting이나 registration 성능을 아직 주장하지 않는다.
- gaussian residual과 shell residual은 scale이 다를 수 있으므로, 현재 단계에서는 직접적인 승패 선언보다 category별 경향 확인에 사용한다.
- 다음 단계는 voxel-level residual normalization 검토와 local registration residual 비교다.
