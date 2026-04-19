# Direct Primitive Validation Plan

이 문서는 stopped semantic relabel branch를 이어서 확장하는 문서가 아니다.
기존 relabel/review workflow는 archive로 보존하고, 새 질문을 direct primitive validation으로 다시 시작한다.

## Stage 1: voxel primitive fitting comparison

목표:

- voxel 단위에서 Gaussian/NDT-style center-based fit과
  GES/GND-like surface primitive scaffold를 같은 출력 포맷으로 비교한다.
- 작은 representative voxel set만 다룬다.
- 아직 registration/system claim은 하지 않는다.

출력:

- `results/direct_primitive_validation/<run>/summary.md`
- `results/direct_primitive_validation/<run>/selected_voxels.csv`
- `results/direct_primitive_validation/<run>/voxel_comparison.csv`
- `results/direct_primitive_validation/<run>/loaded_files.txt`

핵심 지표:

- Gaussian fit residual
- voxel center bias
- surface/shell consistency residual
- point count
- sparsity / degeneracy indicator
- coarse category tag

현재 구현 상태:

- 완료: representative voxel set에 대한 raw residual export
- 완료: residual normalization quickcheck
- 제한: shell 쪽은 아직 exact GES/GND fitting이 아니라 `PCA + quantile scale + L_p shell` proxy다.

## Stage 2: local registration residual comparison

빠른 Stage-2는 full registration이 아니라 small offline quickcheck로 정의한다.

예상 범위:

- local neighborhood를 deterministic reference/source split으로 분리
- nominal vs small perturbation에서 model score delta 비교
- translation / small rotation perturbation에 대한 분별력 비교

현재 구현 상태:

- 완료: selected voxel neighborhood quickcheck
- 출력:
  - `voxel_comparison_normalized.csv`
  - `registration_quickcheck.csv`
  - `selected_cases.csv`
- 아직 미완료:
  - multi-frame registration benchmark
  - frame-to-local-map residual heatmap
  - exact primitive optimization

## Stage 3: system integration

Stage-1과 quick Stage-2가 모두 설득력 있을 때만 고려한다.

예상 범위:

- exact primitive fitting 강화
- local map representation 교체 실험
- 그 다음에야 FAST-LIO/BALM-style integration 검토

## 명시적 비연속성

- archived semantic relabel branch는 continuation 대상이 아니다.
- 이번 branch의 성공 기준은 semantic relabel precision이 아니라
  primitive residual 관점의 직접 비교 근거다.
