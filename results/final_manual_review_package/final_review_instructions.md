# Final Semantic Review Instructions

이 문서는 `junction_like_mixed` scored relabeling에 대한 마지막 semantic validity gate입니다.
이 검토가 끝나면 다음 단계는 둘 중 하나만 선택합니다.

- `CONTINUE`: exact primitive/residual implementation 쪽으로 진행
- `PIVOT` 또는 `STOP`: 이 branch는 더 진행하지 않음

## Open These First

1. `contact_sheets/scored_only_relabeled_contact_sheet.png`
2. `contact_sheets/hard_only_relabeled_contact_sheet.png`
3. `scenes/full0/groups/scored_only_relabeled/contact_sheet.png`
4. `scenes/full153/groups/scored_only_relabeled/contact_sheet.png`
5. `scenes/full95/groups/scored_only_relabeled/contact_sheet.png`
6. `annotation_template.csv`
7. `decision_template.md`

## Exact Inspection Order

1. 먼저 top-level `scored_only`와 `hard_only` contact sheet를 비교합니다.
2. 그다음 `full0`의 `scored_only` contact sheet와 per-voxel overlay preview를 봅니다.
3. 그다음 `full153`을 같은 방식으로 봅니다.
4. 그다음 `full95`를 같은 방식으로 봅니다.
5. 각 voxel에 대해 `annotation_template.csv`를 채웁니다.
6. 마지막으로 `decision_template.md`에 `continue / pivot / stop` 결론을 적습니다.

## Labels To Assign

각 voxel에는 아래 중 하나만 선택합니다.

- `junction_like_mixed`
- `planar_boundary`
- `ambiguous`
- `noise_or_bad_case`

가능하면 `confidence`도 함께 채웁니다.

## Practical Review Rule

- `CONTINUE`
  - 대부분의 `scored_only` voxel이 시각적으로 설득력 있는 `junction_like_mixed`로 보일 때
  - 실무 기준: `16`개 `scored_only` 중 대략 `10`개 이상이 valid
- `PIVOT`
  - 결과가 섞여 있거나, ambiguous가 많거나, semantic gain이 불확실할 때
  - 실무 기준: 대략 `6-9`개가 valid이거나 ambiguous가 다수
- `STOP`
  - 대부분이 ordinary planar boundary, noise, superficial relabeling으로 보일 때
  - 실무 기준: valid가 `5`개 이하

## What Counts As Valid

- neighborhood와 overlay preview에서 단순 planar boundary 확장보다 다방향 mixed / junction 구조가 더 설득력 있게 보이는 경우
- `hard_only`보다 `scored_only`가 전반적으로 더 convincing해 보이는 경우

## What Does Not Count As Success

- numeric gain만으로 성공으로 판단하지 않습니다
- 이미 interesting했던 planar voxel의 semantic reinterpretation만 많다면 `CONTINUE` 근거로 부족합니다

## Use These Files For Borderline Cases

- per-voxel `overlay_preview.png`
- per-voxel `neighborhood_preview.png`
- `annotation_template.csv`에 기록된 original PCD paths
- `source/cross_scene_summary.md`

## Final Note

이 문서는 다음 연구 단계로 넘어가기 전의 마지막 semantic validity gate입니다.
이 검토에서 `CONTINUE`가 나오지 않으면, exact primitive/residual implementation이나 FAST-LIO 쪽 작업은 시작하지 않습니다.
