# Implementation TODO

## Stage 1. Offline voxel morphology analysis

- [x] catkin workspace 루트 정리
- [x] `ges_voxel_mapping` 패키지 추가
- [x] 단일 PCD / PCD 디렉토리 로더 추가
- [x] voxel bucket collector 추가
- [x] voxel PCA / morphology metric 추가
- [x] CSV / summary output 추가
- [ ] corner-like voxel 자동 우선 추출 규칙 튜닝
- [ ] 샘플 데이터셋별 config preset 추가

## Stage 2. Gaussian voxel fitting baseline

- [x] voxel mean / covariance 계산
- [x] 평균 point Mahalanobis metric 계산
- [x] voxel center Mahalanobis metric 계산
- [ ] NDT-style score normalization 정리
- [ ] Gaussian score heatmap/debug output 추가

## Stage 3. GES/GND-style surface primitive prototype

- [x] single primitive per voxel
- [x] PCA local frame 기반 shell residual proxy
- [x] shape exponent `p` 설정 가능
- [x] quantile 기반 axis scale 추정
- [ ] robust axis fitting 개선
- [ ] exact GES/GND fitting solver 초안
- [ ] shell orientation/offset bias 분석

## Stage 4. Surface-consistency residual evaluation

- [x] voxel center residual vs observed point residual 비교
- [x] top interesting voxels CSV 저장
- [ ] voxel-local slice export
- [ ] point-wise residual histogram 저장
- [ ] pseudo-color debug point cloud 저장

## Stage 5. Local registration / local mapping comparison

- [ ] frame-to-local-map offline registration benchmark
- [ ] Gaussian residual vs shell residual pose perturbation sweep
- [ ] local map fidelity 지표 정의
- [ ] scan subset replay pipeline 추가

## Stage 6. FAST-LIO ROS1 integration

- [ ] FAST-LIO dependency bootstrap 자동화
- [ ] FAST-LIO map primitive abstraction layer 추가
- [ ] point-to-plane branch와 분리된 residual interface 추가
- [ ] GES/GND voxel residual prototype 연결
- [ ] runtime debug topic / CSV logging 추가

## Stage 7. BALM-style later extension

- [ ] adaptive voxelization 실험
- [ ] local pose-only BA prototype
- [ ] voxel primitive refinement loop

## 지금 당장 하지 않을 것

- [ ] loop closure
- [ ] place recognition
- [ ] global BA
- [ ] mixture primitive per voxel
- [ ] full end-to-end all-in-one optimizer
