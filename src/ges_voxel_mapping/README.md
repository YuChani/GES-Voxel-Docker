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
