# Junction-Like Mixed Multi-Scene Review Summary

## Aggregate Counts

- shared relabeled voxels: 11
- scored-only relabeled voxels: 16
- hard-only relabeled voxels: 2
- scored-only top-k voxels: 9
- hard-only top-k voxels: 0

## Per-Scene Summary

### full0.pcd

- scene path: `/home/chani/personal/CVRL_SLAM_Docker/GES-Voxel-Docker/prev/BALM/datas/benchmark_realworld/full0.pcd`
- top-k: baseline combined=32, planar=126; hard combined=35, planar=123; scored combined=40, planar=119
- groups: shared=8 (scored_topk=3), scored_only=5 (scored_topk=5), hard_only=2 (hard_topk=0)
- shared ids: 6_5_-1,4_-13_4,5_-13_4,5_12_-1,5_13_-1,6_13_-1,6_13_1,6_5_-2
- scored-only ids: 0_-10_6,5_4_4,-2_-8_4,6_7_-2,1_-10_4
- hard-only ids: 6_12_0,6_12_1
- inspect first:
  - `scenes/full0/groups/scored_only_relabeled/combined_neighborhood_radius2.pcd`
  - `scenes/full0/groups/scored_only_relabeled/annotation_template.csv`
  - `scenes/full0/groups/hard_only_relabeled/combined_neighborhood_radius2.pcd`
  - `scenes/full0/groups/hard_only_relabeled/annotation_template.csv`
  - `scenes/full0/scene_summary.md`
- annotation template: `scenes/full0/annotation_template.csv`

### full153.pcd

- scene path: `/home/chani/personal/CVRL_SLAM_Docker/GES-Voxel-Docker/prev/BALM/datas/benchmark_realworld/full153.pcd`
- top-k: baseline combined=12, planar=143; hard combined=14, planar=138; scored combined=18, planar=134
- groups: shared=3 (scored_topk=3), scored_only=7 (scored_topk=3), hard_only=0 (hard_topk=0)
- shared ids: -3_9_-1,-6_-1_5,-3_9_0
- scored-only ids: 6_2_2,1_6_4,-1_9_-1,-1_8_-2,2_5_6,2_6_4,7_1_2
- hard-only ids: (none)
- inspect first:
  - `scenes/full153/groups/scored_only_relabeled/combined_neighborhood_radius2.pcd`
  - `scenes/full153/groups/scored_only_relabeled/annotation_template.csv`
  - `scenes/full153/groups/hard_only_relabeled/combined_neighborhood_radius2.pcd`
  - `scenes/full153/groups/hard_only_relabeled/annotation_template.csv`
  - `scenes/full153/scene_summary.md`
- annotation template: `scenes/full153/annotation_template.csv`

### full95.pcd

- scene path: `/home/chani/personal/CVRL_SLAM_Docker/GES-Voxel-Docker/prev/BALM/datas/benchmark_realworld/full95.pcd`
- top-k: baseline combined=14, planar=163; hard combined=15, planar=163; scored combined=16, planar=162
- groups: shared=0 (scored_topk=0), scored_only=4 (scored_topk=1), hard_only=0 (hard_topk=0)
- shared ids: (none)
- scored-only ids: -9_-6_2,-8_0_5,-8_3_0,-8_4_0
- hard-only ids: (none)
- inspect first:
  - `scenes/full95/groups/scored_only_relabeled/combined_neighborhood_radius2.pcd`
  - `scenes/full95/groups/scored_only_relabeled/annotation_template.csv`
  - `scenes/full95/groups/hard_only_relabeled/combined_neighborhood_radius2.pcd`
  - `scenes/full95/groups/hard_only_relabeled/annotation_template.csv`
  - `scenes/full95/scene_summary.md`
- annotation template: `scenes/full95/annotation_template.csv`

