# Direct Primitive Reset Note

## 상태 전환

- 기존 `junction_like_mixed` semantic relabel / review line은 `STOP` 판정을 받고 종료했다.
- 종료 의미는 삭제가 아니라 archive다.
- 따라서 기존 `results/final_manual_review_package/`, `results/context_refinement*`,
  `results/context_refinement_scored*` 결과와 review artifact는 traceability를 위해 그대로 보존한다.

## 왜 중단했는가

- numeric gain은 있었지만, final manual review 기준으로 semantic jump가 충분히 설득력 있지 않았다.
- 많은 사례가 새로운 mixed/junction 발견이라기보다 planar boundary의 reinterpretation에 가까웠다.
- 따라서 이 line을 exact primitive/residual 구현으로 바로 확장하는 것은 연구 리스크가 컸다.

## 새 연구 질문

> GES/GND-like surface primitive가 surface-sampled LiDAR voxel을
> Gaussian/NDT-style center-based model보다 더 잘 설명하는가?

## 지금부터 보존되는 것

- stopped relabel branch의 모든 summary/review/export 결과
- 기존 offline PCD loader와 voxel morphology analyzer
- multi-scene review package와 final decision artifact

## 지금부터 out of scope

- `junction_like_mixed` heuristic extension
- score tuning
- semantic relabel refinement
- FAST-LIO integration
- BALM-style system integration

## 새 worktree 원칙

- direct primitive validation은 별도 Stage-1 scaffold로 분리한다.
- Stage-1은 voxel-level primitive fitting comparison까지만 다룬다.
- registration/system 단계는 Stage-1과 Stage-2 결과가 충분히 정당화될 때만 진행한다.
