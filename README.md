# GES-Voxel-Docker

ROS1 Noetic 기반으로 GES/GND 계열 voxel surface primitive 아이디어를 오프라인부터 검증하기 위한 구현 저장소다. 현재 우선순위는 `FAST_LIO` 재작성이나 full SLAM이 아니라, `offline voxel morphology analysis -> Gaussian baseline -> GES/GND-style surface primitive proxy -> residual 비교`를 재현 가능하게 만드는 것이다.

## 현재 저장소 상태

- `src/FAST_LIO`: 기존 ROS1 FAST-LIO 패키지 사본
- `prev/BALM`: 참고용 BALM 코드
- `src/ges_voxel_mapping`: 새 오프라인 실험 패키지
- `dockerfile/fast-lio.Dockerfile`: ROS1 Noetic 개발/실험용 Dockerfile
- `docker-compose/fast-lio_compose.yml`: 개발용 compose
- `TODO.md`: 단계별 구현 TODO

## 중요한 운영 원칙

- 기본 빌드는 오프라인 MVP에 집중한다.
- `src/FAST_LIO`는 현재 기본 워크스페이스 빌드에서 제외되어 있다.
  - 이유: 로컬 저장소에 `ikd-Tree`, `livox_ros_driver`가 아직 포함되어 있지 않아 즉시 빌드가 깨지기 때문이다.
  - 오프라인 실험을 먼저 안정화한 뒤 `scripts/bootstrap_fastlio_deps.sh`로 FAST-LIO 의존성을 채워 통합 단계로 넘어간다.
- 새 코드는 가능한 한 `src/ges_voxel_mapping` 안에 격리한다.

## 워크스페이스 구조

이 저장소 루트는 catkin workspace 루트로 사용한다.

- workspace root: 현재 저장소 루트
- package root:
  - `src/ges_voxel_mapping`
  - `src/FAST_LIO` (기본 빌드에서는 `CATKIN_IGNORE`)

## 빠른 시작

### 1. 로컬 빌드

오프라인 voxel morphology 분석의 기본 경로는 이제 Python이 아니라 standalone C++ 실행 파일이다.

```bash
./scripts/build_offline.sh
```

이 빌드는 `src/ges_voxel_mapping`만 standalone CMake로 컴파일하므로, 호스트에 `/opt/ros/noetic`가 없어도 된다.

Noetic/catkin 통합은 나중 단계다. 현재 `src/FAST_LIO`는 `CATKIN_IGNORE`로 기본 catkin 빌드에서 제외되어 있다.

### 2. 첫 실험 실행

기본 runner는 `auto`이며, C++를 먼저 시도하고 실패할 때만 Python fallback으로 내려간다.

단일 PCD, C++ 강제 실행:

```bash
./scripts/run_voxel_morphology.sh \
  --input /abs/path/to/map.pcd \
  --mode single \
  --runner cpp \
  --output "$(pwd)/results/voxel_morphology_cpp/map_name"
```

단일 PCD, Python fallback 강제 실행:

```bash
./scripts/run_voxel_morphology.sh \
  --input /abs/path/to/map.pcd \
  --mode single \
  --runner python \
  --output "$(pwd)/results/voxel_morphology_python_fallback/map_name"
```

PCD 디렉토리 누적 분석:

```bash
./scripts/run_voxel_morphology.sh \
  --input /abs/path/to/pcd_dir \
  --mode aggregate \
  --runner cpp \
  --output "$(pwd)/results/voxel_morphology_cpp"
```

PCD 디렉토리 batch 분석:

```bash
./scripts/run_voxel_morphology.sh \
  --input /abs/path/to/pcd_dir \
  --mode batch \
  --runner cpp \
  --output "$(pwd)/results/voxel_morphology_cpp_batch"
```

출력 분리는 아래처럼 유지하는 것을 권장한다.

- C++: `results/voxel_morphology_cpp/...`
- Python fallback: `results/voxel_morphology_python_fallback/...`

리포 안에 현재 실제로 확인된 대표 샘플은 아래 경로다.

```bash
./scripts/run_voxel_morphology.sh \
  --input ./prev/BALM/datas/benchmark_realworld/full0.pcd \
  --mode single \
  --runner cpp \
  --output "$(pwd)/results/voxel_morphology_cpp/full0"
```

위 명령은 기본적으로 아래 디렉토리를 만든다.

```bash
results/voxel_morphology_cpp/full0/
```

주의:

- 현재 로컬 체크아웃에는 `prev/BALM/data/...`가 없고, 실제 샘플은 `prev/BALM/datas/...` 아래에 있다.
- 따라서 이 저장소에서 바로 재현할 때는 `prev/BALM/datas/benchmark_realworld/full0.pcd`를 사용하면 된다.

### 3. Docker 사용

```bash
./run_build.sh
```

환경 변수로 제어 가능하다:

```bash
BUILD_FAST_LIO=0 IMAGE_NAME=ges-voxel-noetic ./run_build.sh
```

## 현재 MVP

현재 구현된 첫 MVP는 다음을 수행한다.

1. 누적 PCD 또는 PCD 디렉토리를 로드
2. voxelization 수행
3. voxel별 PCA/eigen-structure 계산
4. Gaussian baseline metric 계산
5. GES/GND-inspired shell primitive proxy residual 계산
6. voxel morphology CSV와 summary 저장
7. Gaussian center preference가 강한 voxel과 shell center penalty가 큰 voxel을 추출
8. interesting voxel들을 개별 PCD로 export

정확한 GES/GND 최적화가 아니라, 먼저 surface-shell consistency가 center likelihood보다 타당한지 빠르게 보는 용도다.

## PCD 가정

현재 오프라인 로더는 아래 common case를 우선 지원한다.

- unorganized `XYZ` PCD
- unorganized `XYZI` PCD
- `x y z intensity ...`처럼 추가 필드가 있는 superset-field binary/ascii PCD

필수 조건:

- `x`, `y`, `z` 필드는 반드시 있어야 한다.
- `intensity` 또는 `i` 필드는 있으면 읽고, 없으면 `0`으로 채운다.
- 추가 필드는 현재 분석에 사용하지 않고 무시한다.

현재 로컬 샘플 `full0.pcd`는 다음과 같은 superset-field binary PCD다.

- `x y z intensity normal_x normal_y normal_z curvature`

## FAST-LIO 재활성화

온라인 통합 단계로 갈 때:

```bash
./scripts/bootstrap_fastlio_deps.sh
```

그 다음:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## 참고 소스

- 로컬 FAST-LIO 사본: [src/FAST_LIO](src/FAST_LIO)
- 오프라인 실험 패키지: [src/ges_voxel_mapping](src/ges_voxel_mapping)
- TODO: [TODO.md](TODO.md)
