# ges_voxel_mapping

`ges_voxel_mapping`은 ROS1 catkin 패키지이지만, 첫 단계는 ROS runtime 의존도를 최소화한 오프라인 실험 패키지로 설계했다.

## 현재 포함된 실행기

- `voxel_morphology_analyzer`
  - 입력:
    - 단일 `.pcd`
    - `.pcd` 디렉토리 aggregate
    - `.pcd` 디렉토리 batch
  - 출력:
    - `voxel_metrics.csv`
    - `interesting_voxels.csv`
    - `summary.txt`
    - `loaded_files.txt`
    - `interesting_voxels/manifest.csv`
    - `interesting_voxels/*.pcd`
- `scripts/run_parameter_sweep.py`
  - C++ offline executable만 사용해서 parameter sweep 실행
  - per-run output: `results/parameter_sweep/runs/<run_id>/`
  - aggregate output:
    - `results/parameter_sweep/sweep_summary.csv`
    - `results/parameter_sweep/mode_comparison.csv`
    - `results/parameter_sweep/summary.md`
- `scripts/run_context_refinement.py`
  - best baseline setting에서 neighborhood-context ranking mode 비교
  - per-run output: `results/context_refinement/runs/<mode>/`
  - aggregate output:
    - `results/context_refinement/comparison.csv`
    - `results/context_refinement/summary.md`
- `scripts/run_junction_refinement.py`
  - `context_hybrid` 대비 `junction_priority` 비교
  - per-run output: `results/context_refinement/junction_runs/<mode>/`
  - aggregate output:
    - `results/context_refinement/junction_comparison.csv`
    - `results/context_refinement/junction_summary.md`
- `scripts/run_junction_mixed_reclassification.py`
  - `planar -> junction_like_mixed` hard-threshold vs scored derived label 비교 실험
  - per-run output: `results/context_refinement_scored/junction_mixed_scored_runs/<mode>/`
  - aggregate output:
    - `results/context_refinement_scored/junction_mixed_scored_comparison.csv`
    - `results/context_refinement_scored/junction_mixed_scored_summary.md`
- `scripts/run_junction_mixed_scored_calibration.py`
  - scored relabel threshold sweep과 relabeled voxel review artifact 생성
  - per-run output: `results/context_refinement_scored_calibration/runs/<mode>/`
  - aggregate output:
    - `results/context_refinement_scored_calibration/scored_calibration_comparison.csv`
    - `results/context_refinement_scored_calibration/scored_calibration_summary.md`
    - `results/context_refinement_scored_calibration/relabeled_voxel_review.csv`
    - `results/context_refinement_scored_calibration/relabeled_voxel_review.md`
- `scripts/run_junction_mixed_scored_review.py`
  - hard-threshold vs scored relabeled voxel의 review/export 전용 step
  - per-run output: `results/context_refinement_scored_review/runs/<mode>/`
  - group export:
    - `results/context_refinement_scored_review/groups/shared_relabeled/`
    - `results/context_refinement_scored_review/groups/scored_only_relabeled/`
    - `results/context_refinement_scored_review/groups/hard_only_relabeled/`
  - aggregate output:
    - `results/context_refinement_scored_review/relabeled_review_summary.csv`
    - `results/context_refinement_scored_review/relabeled_review_summary.md`

## 현재 primitive 구현 수준

현재 GES/GND primitive는 정확한 논문형 최적화가 아니라 다음 근사로 시작한다.

1. voxel 내부 점들로 PCA local frame 계산
2. 각 축 방향 scale을 절대좌표 quantile로 추정
3. `L_p` shell 위상으로 surface consistency residual 계산

즉, "center likelihood"가 아니라 "surface shell consistency"가 중심인 첫 proxy다.

## 다음 확장 포인트

- exact GES/GND fitting
- residual heatmap export
- frame-to-local-map registration benchmark
- FAST-LIO local map primitive 교체

## 실행 예시

단일 PCD:

```bash
./scripts/build_offline.sh
./build/ges_voxel_mapping_cpp/voxel_morphology_analyzer \
  --input ./prev/BALM/datas/benchmark_realworld/full0.pcd \
  --mode single \
  --output ./results/voxel_morphology_cpp/full0_default \
  --voxel-size 1.0 \
  --min-points-per-voxel 20 \
  --shape-exponent 1.2 \
  --axis-scale-quantile 0.9 \
  --ranking-mode score_only \
  --save-top-k 200 \
  --export-interesting-voxels true \
  --export-top-k-pcd 30
```

wrapper script 사용:

```bash
./scripts/run_voxel_morphology.sh \
  --input ./prev/BALM/datas/benchmark_realworld/full0.pcd \
  --mode single \
  --runner cpp \
  --output ./results/voxel_morphology
```

디렉토리 누적:

```bash
./scripts/run_voxel_morphology.sh \
  --input ./prev/BALM/datas/benchmark_realworld \
  --mode aggregate \
  --runner cpp \
  --output ./results/voxel_morphology_aggregate
```

디렉토리 batch:

```bash
./scripts/run_voxel_morphology.sh \
  --input ./prev/BALM/datas/benchmark_realworld \
  --mode batch \
  --runner cpp \
  --output ./results/voxel_morphology_batch
```

parameter sweep:

```bash
./scripts/build_offline.sh
python3 ./scripts/run_parameter_sweep.py \
  --input ./prev/BALM/datas/benchmark_realworld/full0.pcd \
  --output-root ./results/parameter_sweep \
  --voxel-sizes 0.8,1.0,1.2 \
  --min-points 15,20 \
  --shape-exponents 1.0,1.4 \
  --axis-scale-quantiles 0.85,0.95 \
  --ranking-modes score_only,corner_priority,nonplanar_priority \
  --top-k 200 \
  --include-baseline
```

context refinement:

```bash
./scripts/build_offline.sh
python3 ./scripts/run_context_refinement.py \
  --input ./prev/BALM/datas/benchmark_realworld/full0.pcd \
  --output-root ./results/context_refinement \
  --voxel-size 1.0 \
  --min-points-per-voxel 15 \
  --shape-exponent 1.0 \
  --axis-scale-quantile 0.95 \
  --top-k 200
```

junction refinement:

```bash
./scripts/build_offline.sh
python3 ./scripts/run_junction_refinement.py \
  --input ./prev/BALM/datas/benchmark_realworld/full0.pcd \
  --output-root ./results/context_refinement \
  --voxel-size 1.0 \
  --min-points-per-voxel 15 \
  --shape-exponent 1.0 \
  --axis-scale-quantile 0.95 \
  --top-k 200
```

junction mixed reclassification:

```bash
./scripts/build_offline.sh
python3 ./scripts/run_junction_mixed_reclassification.py \
  --input ./prev/BALM/datas/benchmark_realworld/full0.pcd \
  --output-root ./results/context_refinement_scored \
  --voxel-size 1.0 \
  --min-points-per-voxel 15 \
  --shape-exponent 1.0 \
  --axis-scale-quantile 0.95 \
  --junction-mixed-min-neighbor-count 10 \
  --junction-mixed-min-cluster-count 3 \
  --junction-mixed-min-score 0.72 \
  --junction-mixed-min-orientation-dispersion 0.48 \
  --junction-mixed-max-dominant-fraction 0.38 \
  --junction-mixed-min-occupancy-asymmetry 0.30 \
  --junction-mixed-min-normal-variation 0.20 \
  --junction-mixed-max-opposite-face-pair-ratio 0.67 \
  --junction-mixed-scored-min-neighbor-count 8 \
  --junction-mixed-scored-min-cluster-count 2 \
  --junction-mixed-scored-min-junction-score 0.30 \
  --junction-mixed-scored-min-orientation-dispersion 0.35 \
  --junction-mixed-scored-max-dominant-fraction 0.62 \
  --junction-mixed-scored-min-occupancy-asymmetry 0.18 \
  --junction-mixed-scored-min-normal-variation 0.10 \
  --junction-mixed-scored-threshold 0.66 \
  --top-k 200
```

junction mixed scored calibration:

```bash
./scripts/build_offline.sh
python3 ./scripts/run_junction_mixed_scored_calibration.py \
  --input ./prev/BALM/datas/benchmark_realworld/full0.pcd \
  --output-root ./results/context_refinement_scored_calibration \
  --voxel-size 1.0 \
  --min-points-per-voxel 15 \
  --shape-exponent 1.0 \
  --axis-scale-quantile 0.95 \
  --score-thresholds 0.58,0.62,0.66,0.70,0.74 \
  --review-threshold 0.66 \
  --junction-mixed-min-neighbor-count 10 \
  --junction-mixed-min-cluster-count 3 \
  --junction-mixed-min-score 0.72 \
  --junction-mixed-min-orientation-dispersion 0.48 \
  --junction-mixed-max-dominant-fraction 0.38 \
  --junction-mixed-min-occupancy-asymmetry 0.30 \
  --junction-mixed-min-normal-variation 0.20 \
  --junction-mixed-max-opposite-face-pair-ratio 0.67 \
  --junction-mixed-scored-min-neighbor-count 8 \
  --junction-mixed-scored-min-cluster-count 2 \
  --junction-mixed-scored-min-junction-score 0.30 \
  --junction-mixed-scored-min-orientation-dispersion 0.35 \
  --junction-mixed-scored-max-dominant-fraction 0.62 \
  --junction-mixed-scored-min-occupancy-asymmetry 0.18 \
  --junction-mixed-scored-min-normal-variation 0.10 \
  --junction-mixed-scored-threshold 0.66 \
  --top-k 200
```

junction mixed scored review/export:

```bash
./scripts/build_offline.sh
python3 ./scripts/run_junction_mixed_scored_review.py \
  --input ./prev/BALM/datas/benchmark_realworld/full0.pcd \
  --output-root ./results/context_refinement_scored_review \
  --voxel-size 1.0 \
  --min-points-per-voxel 15 \
  --shape-exponent 1.0 \
  --axis-scale-quantile 0.95 \
  --review-threshold 0.66 \
  --top-k 200 \
  --neighborhood-radius 2 \
  --junction-mixed-min-neighbor-count 10 \
  --junction-mixed-min-cluster-count 3 \
  --junction-mixed-min-score 0.72 \
  --junction-mixed-min-orientation-dispersion 0.48 \
  --junction-mixed-max-dominant-fraction 0.38 \
  --junction-mixed-min-occupancy-asymmetry 0.30 \
  --junction-mixed-min-normal-variation 0.20 \
  --junction-mixed-max-opposite-face-pair-ratio 0.67 \
  --junction-mixed-scored-min-neighbor-count 8 \
  --junction-mixed-scored-min-cluster-count 2 \
  --junction-mixed-scored-min-junction-score 0.30 \
  --junction-mixed-scored-min-orientation-dispersion 0.35 \
  --junction-mixed-scored-max-dominant-fraction 0.62 \
  --junction-mixed-scored-min-occupancy-asymmetry 0.18 \
  --junction-mixed-scored-min-normal-variation 0.10 \
  --junction-mixed-scored-threshold 0.66
```

junction mixed scored multi-scene review/annotation export:

```bash
./scripts/build_offline.sh
python3 ./scripts/run_junction_mixed_scored_multiscene_review.py \
  --input-root ./prev/BALM/datas/benchmark_realworld \
  --output-root ./results/context_refinement_scored_multiscene_review \
  --voxel-size 1.0 \
  --min-points-per-voxel 15 \
  --shape-exponent 1.0 \
  --axis-scale-quantile 0.95 \
  --review-threshold 0.66 \
  --top-k 200 \
  --neighborhood-radius 2 \
  --junction-mixed-min-neighbor-count 10 \
  --junction-mixed-min-cluster-count 3 \
  --junction-mixed-min-score 0.72 \
  --junction-mixed-min-orientation-dispersion 0.48 \
  --junction-mixed-max-dominant-fraction 0.38 \
  --junction-mixed-min-occupancy-asymmetry 0.30 \
  --junction-mixed-min-normal-variation 0.20 \
  --junction-mixed-max-opposite-face-pair-ratio 0.67 \
  --junction-mixed-scored-min-neighbor-count 8 \
  --junction-mixed-scored-min-cluster-count 2 \
  --junction-mixed-scored-min-junction-score 0.30 \
  --junction-mixed-scored-min-orientation-dispersion 0.35 \
  --junction-mixed-scored-max-dominant-fraction 0.62 \
  --junction-mixed-scored-min-occupancy-asymmetry 0.18 \
  --junction-mixed-scored-min-normal-variation 0.10 \
  --junction-mixed-scored-threshold 0.66
```

기본 동작은 `full0.pcd`를 anchor로 사용하고, 같은 디렉터리의 `full*.pcd` 전체 point count를 읽어
25% / 75% 분위수에 가까운 두 장면을 자동 선택합니다. 결과는
`results/context_refinement_scored_multiscene_review/` 아래에 scene selection, per-scene review,
group manifest, annotation template, cross-scene summary로 저장됩니다.

final manual review package with automatic PNG previews:

```bash
python3 ./scripts/build_final_manual_review_package.py \
  --review-root ./results/context_refinement_scored_multiscene_review \
  --output-root ./results/final_manual_review_package \
  --panel-size 320 \
  --thumbnail-width 360 \
  --contact-columns 2
```

이 스크립트는 기존 multi-scene review 결과를 재사용해
`results/final_manual_review_package/` 아래에 다음을 생성합니다.

- `review_index.md`
- `annotation_template.csv`
- `decision_template.md`
- scene/group/voxel별 `voxel_preview.png`, `neighborhood_preview.png`, `overlay_preview.png`
- top-level / scene-level group contact sheet PNG

기본적으로 Open3D headless offscreen 렌더를 먼저 시도하고, 현재 환경에서 불가능하면
deterministic projection PNG로 자동 fallback 합니다.

## ranking/filter mode

- `score_only`
  - 기존 `gaussian_center_advantage + shell_center_penalty`
- `corner_priority`
  - `corner_like`/`volumetric` voxel에 가산점, `planar`/`linear` voxel에 감점
  - morphology statistic 기반 thin planar penalty를 약하게 적용
- `nonplanar_priority`
  - scattering 보상을 더 키우고 planarity 감점을 더 강화
  - morphology statistic 기반 thin planar penalty를 강하게 적용
- `context_face_support`
  - face-neighbor occupancy와 opposite-face pair support가 강한 voxel을 감점
- `context_normal_variation`
  - 이웃 voxel normal variation이 큰 voxel에 가산점
- `context_asymmetry`
  - opposite-face occupancy asymmetry가 큰 voxel에 가산점
- `context_hybrid`
  - `nonplanar_priority` 위에 face-support penalty와 corner-context bonus를 함께 적용
- `junction_priority`
  - 2-ring neighborhood normal cluster / orientation dispersion 기반 junction score를 추가 적용
- `junction_mixed_priority`
  - `junction_like_mixed` derived label과 junction score를 함께 사용하는 보수적 재분류 실험용 mode
- `junction_mixed_scored`
  - 기존 hard-threshold 대신 lightweight weighted score로 `planar -> junction_like_mixed`를 재분류하는 mode
