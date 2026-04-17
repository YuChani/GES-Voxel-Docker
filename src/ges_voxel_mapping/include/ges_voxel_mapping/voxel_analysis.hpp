#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace ges_voxel_mapping
{

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

struct AnalysisConfig
{
  std::string input_path;
  std::string output_dir;
  std::string input_mode = "auto";
  double voxel_size = 1.0;
  int min_points_per_voxel = 20;
  int max_points = -1;
  int max_voxels = -1;
  double gaussian_regularization = 1e-3;
  double shape_exponent = 1.2;
  double axis_scale_quantile = 0.9;
  double axis_scale_min = 0.05;
  std::string ranking_mode = "score_only";
  int save_top_k = 200;
  bool export_interesting_voxels = true;
  int export_top_k_pcd = 30;
};

struct VoxelKey
{
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelKey& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey& key) const noexcept;
};

struct VoxelBucket
{
  VoxelKey key;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> points;
};

struct VoxelStats
{
  std::size_t num_points = 0;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
  Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
  double linearity = 0.0;
  double planarity = 0.0;
  double scattering = 0.0;
  double anisotropy = 0.0;
  double omnivariance = 0.0;
  std::string label = "unknown";
};

struct GaussianMetrics
{
  double average_mahalanobis2 = 0.0;
  double voxel_center_mahalanobis2 = 0.0;
  double average_euclidean_distance = 0.0;
  double center_advantage = 0.0;
};

struct ShellMetrics
{
  double average_surface_residual = 0.0;
  double voxel_center_surface_residual = 0.0;
  double average_surface_radius = 0.0;
  double center_penalty = 0.0;
};

struct ContextMetrics
{
  int occupied_face_count = 0;
  double occupied_face_ratio = 0.0;
  double opposite_face_pair_ratio = 0.0;
  double normal_variation = 0.0;
  double occupancy_asymmetry = 0.0;
  double planar_context_penalty = 0.0;
  double corner_context_bonus = 0.0;
};

struct VoxelMetrics
{
  VoxelKey key;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  VoxelStats stats;
  GaussianMetrics gaussian;
  ShellMetrics shell;
  ContextMetrics context;
};

AnalysisConfig LoadAnalysisConfig(const std::string& path);
std::vector<std::string> CollectInputFiles(const std::string& input_path);
PointCloud::Ptr LoadPointCloudFile(const std::string& path, std::string* file_description = nullptr);
PointCloud::Ptr LoadInputCloud(const AnalysisConfig& config, std::vector<std::string>* loaded_files);
std::vector<VoxelMetrics> RunVoxelMorphologyAnalysis(const PointCloud& cloud, const AnalysisConfig& config);
void SaveAnalysisResults(
  const std::vector<VoxelMetrics>& metrics,
  const AnalysisConfig& config,
  std::size_t input_points,
  const std::vector<std::string>& loaded_files,
  const PointCloud* input_cloud);

}  // namespace ges_voxel_mapping
