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
./scripts/run_voxel_morphology.sh \
  --input ./prev/BALM/datas/benchmark_realworld/full0.pcd \
  --mode single \
  --output ./results/voxel_morphology
```

디렉토리 누적:

```bash
./scripts/run_voxel_morphology.sh \
  --input ./prev/BALM/datas/benchmark_realworld \
  --mode aggregate \
  --output ./results/voxel_morphology_aggregate
```

디렉토리 batch:

```bash
./scripts/run_voxel_morphology.sh \
  --input ./prev/BALM/datas/benchmark_realworld \
  --mode batch \
  --output ./results/voxel_morphology_batch
```
