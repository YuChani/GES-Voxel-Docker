#pragma once

#include "ges_voxel_mapping/voxel_analysis.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace ges_voxel_mapping
{

struct PrimitiveValidationConfig
{
  std::string input_path;
  std::string output_dir = "results/direct_primitive_validation";
  double voxel_size = 1.0;
  int min_points_per_voxel = 15;
  int max_points = -1;
  int max_voxels = 12;
  int per_category_limit = 4;
  double gaussian_regularization = 1e-3;
  double shell_axis_scale_quantile = 0.90;
  double shell_axis_scale_min = 0.05;
  double shell_shape_exponent = 1.0;
  std::string selection_mode = "auto_small";
  int registration_neighborhood_voxels = 1;
  int registration_min_points = 24;
  double registration_translation_step_ratio = 0.10;
  double registration_rotation_degrees = 5.0;
};

struct PrimitiveVoxelComparison
{
  VoxelKey key;
  Eigen::Vector3d voxel_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
  std::size_t point_count = 0;
  std::string base_label = "unknown";
  std::string selection_tag = "unselected";
  std::string selection_note;
  double linearity = 0.0;
  double planarity = 0.0;
  double scattering = 0.0;
  double anisotropy = 0.0;
  double omnivariance = 0.0;
  double gaussian_average_mahalanobis2 = 0.0;
  double gaussian_voxel_center_mahalanobis2 = 0.0;
  double gaussian_average_center_distance = 0.0;
  double gaussian_condition_ratio = 0.0;
  double surface_normal_rms = 0.0;
  double shell_average_residual = 0.0;
  double shell_voxel_center_residual = 0.0;
  double shell_average_radius = 0.0;
  Eigen::Vector3d shell_axis_scales = Eigen::Vector3d::Zero();
  double shell_axis_condition = 0.0;
  double characteristic_spread_scale = 0.0;
  double shell_scale_mean = 0.0;
  double shell_geometric_average_residual = 0.0;
  double shell_geometric_center_residual = 0.0;
  double gaussian_normalized_residual_by_voxel = 0.0;
  double shell_normalized_residual_by_voxel = 0.0;
  double gaussian_normalized_residual_by_spread = 0.0;
  double shell_normalized_residual_by_spread = 0.0;
  double normalized_residual_gap = 0.0;
  double sparsity_indicator = 0.0;
  int local_neighborhood_point_count = 0;
  bool degenerate = false;
};

struct PrimitiveRegistrationQuickcheck
{
  VoxelKey key;
  std::string voxel_id;
  std::string selection_tag;
  std::string base_label;
  std::string perturbation;
  int neighborhood_point_count = 0;
  int reference_point_count = 0;
  int source_point_count = 0;
  double perturbation_translation_norm = 0.0;
  double perturbation_rotation_degrees = 0.0;
  double gaussian_nominal_score = 0.0;
  double gaussian_perturbed_score = 0.0;
  double gaussian_delta = 0.0;
  double shell_nominal_score = 0.0;
  double shell_perturbed_score = 0.0;
  double shell_delta = 0.0;
  double delta_advantage = 0.0;
  bool shell_better_discrimination = false;
};

struct PrimitiveValidationRun
{
  std::vector<PrimitiveVoxelComparison> comparisons;
  std::vector<PrimitiveRegistrationQuickcheck> registration_quickchecks;
};

PointCloud::Ptr LoadPrimitiveValidationCloud(
  const PrimitiveValidationConfig& config,
  std::vector<std::string>* loaded_files);

PrimitiveValidationRun RunDirectPrimitiveValidation(
  const PointCloud& cloud,
  const PrimitiveValidationConfig& config);

void SaveDirectPrimitiveValidationResults(
  const PrimitiveValidationRun& run,
  const PrimitiveValidationConfig& config,
  std::size_t input_points,
  const std::vector<std::string>& loaded_files);

}  // namespace ges_voxel_mapping
