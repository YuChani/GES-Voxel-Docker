#include "ges_voxel_mapping/voxel_analysis.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Eigenvalues>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace ges_voxel_mapping
{
namespace
{

constexpr double kEpsilon = 1e-9;

std::string ToLower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
  return text;
}

VoxelKey MakeVoxelKey(const Eigen::Vector3d& point, double voxel_size)
{
  return VoxelKey{
    static_cast<int>(std::floor(point.x() / voxel_size)),
    static_cast<int>(std::floor(point.y() / voxel_size)),
    static_cast<int>(std::floor(point.z() / voxel_size))};
}

Eigen::Vector3d VoxelCenterFromKey(const VoxelKey& key, double voxel_size)
{
  return Eigen::Vector3d(
    (static_cast<double>(key.x) + 0.5) * voxel_size,
    (static_cast<double>(key.y) + 0.5) * voxel_size,
    (static_cast<double>(key.z) + 0.5) * voxel_size);
}

const pcl::PCLPointField* FindField(const pcl::PCLPointCloud2& cloud, const std::vector<std::string>& names)
{
  for (const auto& name : names)
  {
    for (const auto& field : cloud.fields)
    {
      if (field.name == name)
      {
        return &field;
      }
    }
  }
  return nullptr;
}

double ReadNumericField(const uint8_t* field_ptr, uint8_t datatype)
{
  switch (datatype)
  {
    case pcl::PCLPointField::INT8:
      return static_cast<double>(*reinterpret_cast<const int8_t*>(field_ptr));
    case pcl::PCLPointField::UINT8:
      return static_cast<double>(*reinterpret_cast<const uint8_t*>(field_ptr));
    case pcl::PCLPointField::INT16:
      return static_cast<double>(*reinterpret_cast<const int16_t*>(field_ptr));
    case pcl::PCLPointField::UINT16:
      return static_cast<double>(*reinterpret_cast<const uint16_t*>(field_ptr));
    case pcl::PCLPointField::INT32:
      return static_cast<double>(*reinterpret_cast<const int32_t*>(field_ptr));
    case pcl::PCLPointField::UINT32:
      return static_cast<double>(*reinterpret_cast<const uint32_t*>(field_ptr));
    case pcl::PCLPointField::FLOAT32:
      return static_cast<double>(*reinterpret_cast<const float*>(field_ptr));
    case pcl::PCLPointField::FLOAT64:
      return *reinterpret_cast<const double*>(field_ptr);
    default:
      throw std::runtime_error("Unsupported PCD numeric datatype");
  }
}

std::string JoinFieldNames(const pcl::PCLPointCloud2& cloud)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < cloud.fields.size(); ++index)
  {
    if (index > 0)
    {
      stream << ';';
    }
    stream << cloud.fields[index].name;
  }
  return stream.str();
}

void MaybeDownsampleByStride(PointCloud& cloud, int max_points)
{
  if (max_points <= 0 || static_cast<int>(cloud.size()) <= max_points)
  {
    return;
  }

  const std::size_t stride = static_cast<std::size_t>(
    std::ceil(static_cast<double>(cloud.size()) / static_cast<double>(max_points)));

  PointCloud sampled;
  sampled.reserve(max_points);
  for (std::size_t idx = 0; idx < cloud.size(); idx += stride)
  {
    sampled.push_back(cloud[idx]);
  }
  cloud.swap(sampled);
}

VoxelStats ComputeVoxelStats(const VoxelBucket& voxel)
{
  VoxelStats stats;
  stats.num_points = voxel.points.size();
  if (voxel.points.empty())
  {
    return stats;
  }

  for (const auto& point : voxel.points)
  {
    stats.mean += point;
  }
  stats.mean /= static_cast<double>(voxel.points.size());

  if (voxel.points.size() >= 2)
  {
    for (const auto& point : voxel.points)
    {
      const Eigen::Vector3d centered = point - stats.mean;
      stats.covariance += centered * centered.transpose();
    }
    stats.covariance /= static_cast<double>(voxel.points.size() - 1);
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(stats.covariance);
  if (solver.info() != Eigen::Success)
  {
    stats.label = "eigensolver_failed";
    return stats;
  }

  const Eigen::Vector3d ascending_values = solver.eigenvalues();
  const Eigen::Matrix3d ascending_vectors = solver.eigenvectors();
  for (int idx = 0; idx < 3; ++idx)
  {
    stats.eigenvalues(idx) = std::max(ascending_values(2 - idx), 0.0);
    stats.eigenvectors.col(idx) = ascending_vectors.col(2 - idx);
  }

  const double l1 = std::max(stats.eigenvalues(0), kEpsilon);
  const double l2 = std::max(stats.eigenvalues(1), 0.0);
  const double l3 = std::max(stats.eigenvalues(2), 0.0);

  stats.linearity = (l1 - l2) / l1;
  stats.planarity = (l2 - l3) / l1;
  stats.scattering = l3 / l1;
  stats.anisotropy = (l1 - l3) / l1;
  stats.omnivariance = std::cbrt(std::max(l1 * l2 * l3, 0.0));

  const double ratio21 = l2 / l1;
  const double ratio31 = l3 / l1;

  if (stats.num_points < 5 || l1 < 1e-8)
  {
    stats.label = "degenerate";
  }
  else if (ratio21 < 0.25 && ratio31 < 0.08)
  {
    stats.label = "linear";
  }
  else if (ratio21 >= 0.25 && ratio31 < 0.12)
  {
    stats.label = "planar";
  }
  else if (ratio31 > 0.30)
  {
    stats.label = "volumetric";
  }
  else
  {
    stats.label = "corner_like";
  }

  return stats;
}

double MahalanobisSquared(
  const Eigen::Vector3d& point,
  const Eigen::Vector3d& mean,
  const Eigen::Matrix3d& covariance_inv)
{
  const Eigen::Vector3d delta = point - mean;
  return delta.transpose() * covariance_inv * delta;
}

GaussianMetrics ComputeGaussianMetrics(
  const VoxelBucket& voxel,
  const VoxelStats& stats,
  double regularization)
{
  GaussianMetrics metrics;

  Eigen::Matrix3d covariance_reg = stats.covariance;
  covariance_reg.diagonal().array() += regularization;
  const Eigen::Matrix3d covariance_inv = covariance_reg.inverse();

  for (const auto& point : voxel.points)
  {
    metrics.average_mahalanobis2 += MahalanobisSquared(point, stats.mean, covariance_inv);
    metrics.average_euclidean_distance += (point - stats.mean).norm();
  }

  const double count = static_cast<double>(std::max<std::size_t>(voxel.points.size(), 1));
  metrics.average_mahalanobis2 /= count;
  metrics.average_euclidean_distance /= count;
  metrics.voxel_center_mahalanobis2 = MahalanobisSquared(voxel.center, stats.mean, covariance_inv);
  metrics.center_advantage = metrics.average_mahalanobis2 - metrics.voxel_center_mahalanobis2;
  return metrics;
}

double ComputeQuantile(std::vector<double> values, double quantile)
{
  if (values.empty())
  {
    return 0.0;
  }

  quantile = std::clamp(quantile, 0.0, 1.0);
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
    std::floor(quantile * static_cast<double>(values.size() - 1)));
  return values[index];
}

double ComputeLpSurfaceRadius(
  const Eigen::Vector3d& local_point,
  const Eigen::Vector3d& axis_scales,
  double shape_exponent)
{
  const double p = std::max(shape_exponent, 0.25);
  double accum = 0.0;
  for (int axis = 0; axis < 3; ++axis)
  {
    const double normalized = std::abs(local_point(axis)) / std::max(axis_scales(axis), kEpsilon);
    accum += std::pow(normalized, p);
  }
  return std::pow(accum, 1.0 / p);
}

ShellMetrics ComputeShellMetrics(
  const VoxelBucket& voxel,
  const VoxelStats& stats,
  double axis_scale_quantile,
  double axis_scale_min,
  double shape_exponent)
{
  ShellMetrics metrics;

  std::vector<double> axis_abs[3];
  axis_abs[0].reserve(voxel.points.size());
  axis_abs[1].reserve(voxel.points.size());
  axis_abs[2].reserve(voxel.points.size());

  std::vector<Eigen::Vector3d> local_points;
  local_points.reserve(voxel.points.size());

  for (const auto& point : voxel.points)
  {
    const Eigen::Vector3d local = stats.eigenvectors.transpose() * (point - stats.mean);
    local_points.push_back(local);
    for (int axis = 0; axis < 3; ++axis)
    {
      axis_abs[axis].push_back(std::abs(local(axis)));
    }
  }

  Eigen::Vector3d axis_scales = Eigen::Vector3d::Constant(axis_scale_min);
  for (int axis = 0; axis < 3; ++axis)
  {
    axis_scales(axis) = std::max(ComputeQuantile(axis_abs[axis], axis_scale_quantile), axis_scale_min);
  }

  for (const auto& local_point : local_points)
  {
    const double radius = ComputeLpSurfaceRadius(local_point, axis_scales, shape_exponent);
    metrics.average_surface_radius += radius;
    metrics.average_surface_residual += std::abs(radius - 1.0);
  }

  const double count = static_cast<double>(std::max<std::size_t>(voxel.points.size(), 1));
  metrics.average_surface_radius /= count;
  metrics.average_surface_residual /= count;

  const Eigen::Vector3d center_local = stats.eigenvectors.transpose() * (voxel.center - stats.mean);
  metrics.voxel_center_surface_residual =
    std::abs(ComputeLpSurfaceRadius(center_local, axis_scales, shape_exponent) - 1.0);
  metrics.center_penalty =
    metrics.voxel_center_surface_residual - metrics.average_surface_residual;

  return metrics;
}

std::string FormatDouble(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  return stream.str();
}

std::string NormalizeRankingMode(std::string mode)
{
  mode = ToLower(std::move(mode));
  if (mode.empty())
  {
    return "score_only";
  }
  if (
    mode != "score_only" &&
    mode != "corner_priority" &&
    mode != "nonplanar_priority" &&
    mode != "context_face_support" &&
    mode != "context_normal_variation" &&
    mode != "context_asymmetry" &&
    mode != "context_hybrid")
  {
    throw std::runtime_error(
      "Unsupported ranking_mode: " + mode +
      ". Use score_only|corner_priority|nonplanar_priority|context_face_support|"
      "context_normal_variation|context_asymmetry|context_hybrid.");
  }
  return mode;
}

bool UsesCornerPriorityBase(const std::string& ranking_mode)
{
  return ranking_mode == "corner_priority";
}

double ComputeBaseInterestingScore(const VoxelMetrics& metric)
{
  return metric.gaussian.center_advantage + metric.shell.center_penalty;
}

double ComputeRankingAdjustment(const VoxelMetrics& metric, const std::string& ranking_mode)
{
  if (ranking_mode == "score_only")
  {
    return 0.0;
  }

  const bool use_corner_priority = UsesCornerPriorityBase(ranking_mode);
  double label_bias = 0.0;
  if (metric.stats.label == "corner_like")
  {
    label_bias = use_corner_priority ? 1.75 : 0.75;
  }
  else if (metric.stats.label == "volumetric")
  {
    label_bias = use_corner_priority ? 0.75 : 0.50;
  }
  else if (metric.stats.label == "planar")
  {
    label_bias = use_corner_priority ? -1.50 : -1.00;
  }
  else if (metric.stats.label == "linear")
  {
    label_bias = use_corner_priority ? -0.50 : -0.25;
  }
  else if (metric.stats.label == "degenerate")
  {
    label_bias = -2.00;
  }

  if (use_corner_priority)
  {
    return label_bias + 1.50 * metric.stats.scattering - 1.25 * metric.stats.planarity;
  }

  return label_bias + 2.25 * metric.stats.scattering - 1.75 * metric.stats.planarity;
}

double ComputeThinPlanarFilterPenalty(const VoxelMetrics& metric, const std::string& ranking_mode)
{
  if (ranking_mode == "score_only")
  {
    return 0.0;
  }

  const auto& stats = metric.stats;
  double confidence = 0.0;
  confidence += std::clamp((stats.planarity - 0.55) / 0.25, 0.0, 1.0);
  confidence += std::clamp((0.14 - stats.scattering) / 0.08, 0.0, 1.0);
  confidence += std::clamp((stats.anisotropy - 0.80) / 0.20, 0.0, 1.0);
  confidence += std::clamp((0.10 - stats.omnivariance) / 0.08, 0.0, 1.0);
  confidence += std::clamp((0.45 - stats.linearity) / 0.25, 0.0, 1.0);
  confidence /= 5.0;

  if (stats.label == "planar")
  {
    confidence *= 1.25;
  }
  else if (stats.label == "corner_like" || stats.label == "volumetric")
  {
    confidence *= 0.50;
  }

  const double strength = UsesCornerPriorityBase(ranking_mode) ? 1.5 : 3.0;
  return strength * confidence;
}

VoxelKey OffsetVoxelKey(const VoxelKey& key, int dx, int dy, int dz)
{
  return VoxelKey{key.x + dx, key.y + dy, key.z + dz};
}

void ComputeContextMetrics(std::vector<VoxelMetrics>* metrics)
{
  if (metrics == nullptr || metrics->empty())
  {
    return;
  }

  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> lookup;
  lookup.reserve(metrics->size());
  for (std::size_t index = 0; index < metrics->size(); ++index)
  {
    lookup[(*metrics)[index].key] = index;
  }

  constexpr int kFaceOffsets[6][3] = {
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
    {0, 0, 1},
    {0, 0, -1}};

  for (auto& metric : *metrics)
  {
    const Eigen::Vector3d normal = metric.stats.eigenvectors.col(2).normalized();

    std::size_t occupied_face_count = 0;
    double normal_variation_sum = 0.0;
    int normal_variation_count = 0;
    double axis_asymmetry_sum = 0.0;
    int opposite_face_pairs = 0;

    for (int axis = 0; axis < 3; ++axis)
    {
      const int positive_index = axis * 2;
      const int negative_index = axis * 2 + 1;

      const VoxelKey positive_key = OffsetVoxelKey(
        metric.key,
        kFaceOffsets[positive_index][0],
        kFaceOffsets[positive_index][1],
        kFaceOffsets[positive_index][2]);
      const VoxelKey negative_key = OffsetVoxelKey(
        metric.key,
        kFaceOffsets[negative_index][0],
        kFaceOffsets[negative_index][1],
        kFaceOffsets[negative_index][2]);

      const auto positive_it = lookup.find(positive_key);
      const auto negative_it = lookup.find(negative_key);

      const VoxelMetrics* positive_neighbor =
        positive_it == lookup.end() ? nullptr : &(*metrics)[positive_it->second];
      const VoxelMetrics* negative_neighbor =
        negative_it == lookup.end() ? nullptr : &(*metrics)[negative_it->second];

      if (positive_neighbor != nullptr)
      {
        ++occupied_face_count;
        const Eigen::Vector3d neighbor_normal = positive_neighbor->stats.eigenvectors.col(2).normalized();
        normal_variation_sum += 1.0 - std::abs(normal.dot(neighbor_normal));
        ++normal_variation_count;
      }
      if (negative_neighbor != nullptr)
      {
        ++occupied_face_count;
        const Eigen::Vector3d neighbor_normal = negative_neighbor->stats.eigenvectors.col(2).normalized();
        normal_variation_sum += 1.0 - std::abs(normal.dot(neighbor_normal));
        ++normal_variation_count;
      }

      const double positive_mass = positive_neighbor != nullptr
        ? static_cast<double>(positive_neighbor->stats.num_points)
        : 0.0;
      const double negative_mass = negative_neighbor != nullptr
        ? static_cast<double>(negative_neighbor->stats.num_points)
        : 0.0;
      const double total_mass = positive_mass + negative_mass;
      axis_asymmetry_sum += total_mass > 0.0 ? std::abs(positive_mass - negative_mass) / total_mass : 0.0;

      if (positive_neighbor != nullptr && negative_neighbor != nullptr)
      {
        ++opposite_face_pairs;
      }
    }

    metric.context.occupied_face_count = static_cast<int>(occupied_face_count);
    metric.context.occupied_face_ratio = static_cast<double>(occupied_face_count) / 6.0;
    metric.context.opposite_face_pair_ratio = static_cast<double>(opposite_face_pairs) / 3.0;
    metric.context.normal_variation =
      normal_variation_count > 0 ? normal_variation_sum / static_cast<double>(normal_variation_count) : 0.0;
    metric.context.occupancy_asymmetry = axis_asymmetry_sum / 3.0;

    const double support_consistency =
      0.55 * metric.context.occupied_face_ratio + 0.45 * metric.context.opposite_face_pair_ratio;
    const double shape_planar_confidence =
      std::clamp(0.65 * metric.stats.planarity + 0.35 * metric.stats.anisotropy, 0.0, 1.0);
    metric.context.planar_context_penalty =
      support_consistency * (1.0 - metric.context.normal_variation) * shape_planar_confidence;
    metric.context.corner_context_bonus =
      1.35 * metric.context.normal_variation +
      1.10 * metric.context.occupancy_asymmetry +
      0.40 * (1.0 - metric.context.opposite_face_pair_ratio);
  }
}

double ComputeContextModeContribution(const VoxelMetrics& metric, const std::string& ranking_mode)
{
  if (ranking_mode == "context_face_support")
  {
    return
      -3.00 * metric.context.planar_context_penalty -
      1.00 * metric.context.occupied_face_ratio -
      0.75 * metric.context.opposite_face_pair_ratio;
  }
  if (ranking_mode == "context_normal_variation")
  {
    return
      3.00 * metric.context.normal_variation +
      0.75 * metric.context.corner_context_bonus -
      1.25 * metric.context.planar_context_penalty;
  }
  if (ranking_mode == "context_asymmetry")
  {
    return
      2.75 * metric.context.occupancy_asymmetry +
      0.50 * (1.0 - metric.context.opposite_face_pair_ratio) -
      1.00 * metric.context.planar_context_penalty;
  }
  if (ranking_mode == "context_hybrid")
  {
    return
      2.50 * metric.context.corner_context_bonus -
      3.25 * metric.context.planar_context_penalty -
      0.75 * metric.context.occupied_face_ratio;
  }
  return 0.0;
}

double ComputeInterestingScore(const VoxelMetrics& metric, const std::string& ranking_mode)
{
  return
    ComputeBaseInterestingScore(metric) +
    ComputeRankingAdjustment(metric, ranking_mode) -
    ComputeThinPlanarFilterPenalty(metric, ranking_mode) +
    ComputeContextModeContribution(metric, ranking_mode);
}

void EnsureDirectory(const fs::path& directory)
{
  if (!directory.empty())
  {
    fs::create_directories(directory);
  }
}

std::string BuildInterestingVoxelScoreString(const VoxelMetrics& metric, const std::string& ranking_mode)
{
  return FormatDouble(ComputeInterestingScore(metric, ranking_mode));
}

void ExportInterestingVoxelClouds(
  const std::vector<VoxelMetrics>& ranked_metrics,
  const AnalysisConfig& config,
  const PointCloud& input_cloud,
  const fs::path& output_dir)
{
  if (!config.export_interesting_voxels)
  {
    return;
  }

  const std::size_t export_limit = std::min<std::size_t>(
    ranked_metrics.size(),
    static_cast<std::size_t>(std::max(0, std::min(config.export_top_k_pcd, config.save_top_k))));
  if (export_limit == 0)
  {
    return;
  }

  const fs::path voxel_dir = output_dir / "interesting_voxels";
  EnsureDirectory(voxel_dir);

  std::map<VoxelKey, std::size_t, bool(*)(const VoxelKey&, const VoxelKey&)> rank_lookup(
    [](const VoxelKey& lhs, const VoxelKey& rhs)
    {
      if (lhs.x != rhs.x)
      {
        return lhs.x < rhs.x;
      }
      if (lhs.y != rhs.y)
      {
        return lhs.y < rhs.y;
      }
      return lhs.z < rhs.z;
    });

  for (std::size_t rank = 0; rank < export_limit; ++rank)
  {
    rank_lookup[ranked_metrics[rank].key] = rank;
  }

  std::vector<PointCloud> per_rank_clouds(export_limit);
  PointCloud combined;

  for (const auto& point : input_cloud.points)
  {
    const Eigen::Vector3d eigen_point(point.x, point.y, point.z);
    const VoxelKey key = MakeVoxelKey(eigen_point, config.voxel_size);
    const auto rank_it = rank_lookup.find(key);
    if (rank_it == rank_lookup.end())
    {
      continue;
    }

    per_rank_clouds[rank_it->second].push_back(point);
    combined.push_back(point);
  }

  {
    pcl::io::savePCDFileBinary((voxel_dir / "interesting_voxels_combined.pcd").string(), combined);
  }

  {
    std::ofstream manifest(voxel_dir / "manifest.csv");
    manifest << "rank,filename,vx,vy,vz,label,num_points,total_score\n";
    for (std::size_t rank = 0; rank < export_limit; ++rank)
    {
      const auto& metric = ranked_metrics[rank];
      const std::string filename =
        "rank_" + std::to_string(rank) +
        "_vx_" + std::to_string(metric.key.x) +
        "_vy_" + std::to_string(metric.key.y) +
        "_vz_" + std::to_string(metric.key.z) + ".pcd";
      manifest << rank << ','
               << filename << ','
               << metric.key.x << ','
               << metric.key.y << ','
               << metric.key.z << ','
               << metric.stats.label << ','
               << metric.stats.num_points << ','
               << BuildInterestingVoxelScoreString(metric, config.ranking_mode) << '\n';
      pcl::io::savePCDFileBinary((voxel_dir / filename).string(), per_rank_clouds[rank]);
    }
  }
}

}  // namespace

std::size_t VoxelKeyHash::operator()(const VoxelKey& key) const noexcept
{
  std::size_t seed = 0;
  seed ^= std::hash<int>{}(key.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

AnalysisConfig LoadAnalysisConfig(const std::string& path)
{
  const YAML::Node node = YAML::LoadFile(path);
  AnalysisConfig config;
  config.input_path = node["input_path"] ? node["input_path"].as<std::string>() : "";
  config.output_dir = node["output_dir"] ? node["output_dir"].as<std::string>() : "results/voxel_morphology";
  config.input_mode = node["input_mode"] ? node["input_mode"].as<std::string>() : "auto";
  config.voxel_size = node["voxel_size"] ? node["voxel_size"].as<double>() : 1.0;
  config.min_points_per_voxel = node["min_points_per_voxel"] ? node["min_points_per_voxel"].as<int>() : 20;
  config.max_points = node["max_points"] ? node["max_points"].as<int>() : -1;
  config.max_voxels = node["max_voxels"] ? node["max_voxels"].as<int>() : -1;
  config.gaussian_regularization =
    node["gaussian_regularization"] ? node["gaussian_regularization"].as<double>() : 1e-3;
  config.shape_exponent = node["shape_exponent"] ? node["shape_exponent"].as<double>() : 1.2;
  config.axis_scale_quantile = node["axis_scale_quantile"] ? node["axis_scale_quantile"].as<double>() : 0.9;
  config.axis_scale_min = node["axis_scale_min"] ? node["axis_scale_min"].as<double>() : 0.05;
  config.ranking_mode = node["ranking_mode"] ? node["ranking_mode"].as<std::string>() : "score_only";
  config.save_top_k = node["save_top_k"] ? node["save_top_k"].as<int>() : 200;
  config.export_interesting_voxels =
    node["export_interesting_voxels"] ? node["export_interesting_voxels"].as<bool>() : true;
  config.export_top_k_pcd = node["export_top_k_pcd"] ? node["export_top_k_pcd"].as<int>() : 30;

  if (config.voxel_size <= 0.0)
  {
    throw std::runtime_error("voxel_size must be positive");
  }
  if (config.min_points_per_voxel < 3)
  {
    throw std::runtime_error("min_points_per_voxel must be >= 3");
  }
  config.ranking_mode = NormalizeRankingMode(config.ranking_mode);
  return config;
}

std::vector<std::string> CollectInputFiles(const std::string& input_path)
{
  const fs::path path(input_path);
  if (!fs::exists(path))
  {
    throw std::runtime_error("Input path does not exist: " + input_path);
  }

  std::vector<std::string> files;
  if (fs::is_regular_file(path))
  {
    if (ToLower(path.extension().string()) != ".pcd")
    {
      throw std::runtime_error("Only .pcd input is supported right now: " + input_path);
    }
    files.push_back(path.string());
  }
  else if (fs::is_directory(path))
  {
    for (const auto& entry : fs::directory_iterator(path))
    {
      if (entry.is_regular_file() && ToLower(entry.path().extension().string()) == ".pcd")
      {
        files.push_back(entry.path().string());
      }
    }
    std::sort(files.begin(), files.end());
  }

  if (files.empty())
  {
    throw std::runtime_error("No .pcd files found under: " + input_path);
  }

  return files;
}

PointCloud::Ptr LoadPointCloudFile(const std::string& path, std::string* file_description)
{
  pcl::PCLPointCloud2 raw_cloud;
  if (pcl::io::loadPCDFile(path, raw_cloud) != 0)
  {
    throw std::runtime_error("Failed to load PCD: " + path);
  }

  const pcl::PCLPointField* x_field = FindField(raw_cloud, {"x"});
  const pcl::PCLPointField* y_field = FindField(raw_cloud, {"y"});
  const pcl::PCLPointField* z_field = FindField(raw_cloud, {"z"});
  const pcl::PCLPointField* intensity_field = FindField(raw_cloud, {"intensity", "i"});

  if (x_field == nullptr || y_field == nullptr || z_field == nullptr)
  {
    throw std::runtime_error("PCD must contain x/y/z fields: " + path);
  }

  PointCloud::Ptr cloud(new PointCloud);
  cloud->reserve(raw_cloud.width * raw_cloud.height);

  for (std::size_t index = 0; index < raw_cloud.width * raw_cloud.height; ++index)
  {
    const uint8_t* point_ptr = &raw_cloud.data[index * raw_cloud.point_step];
    PointT point;
    point.x = static_cast<float>(ReadNumericField(point_ptr + x_field->offset, x_field->datatype));
    point.y = static_cast<float>(ReadNumericField(point_ptr + y_field->offset, y_field->datatype));
    point.z = static_cast<float>(ReadNumericField(point_ptr + z_field->offset, z_field->datatype));
    point.intensity = intensity_field != nullptr
      ? static_cast<float>(ReadNumericField(point_ptr + intensity_field->offset, intensity_field->datatype))
      : 0.0f;

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    {
      continue;
    }
    cloud->push_back(point);
  }

  if (file_description != nullptr)
  {
    std::ostringstream description;
    description << path
                << " | points=" << cloud->size()
                << " | fields=" << JoinFieldNames(raw_cloud)
                << " | intensity=" << (intensity_field != nullptr ? "yes" : "no");
    *file_description = description.str();
  }

  return cloud;
}

PointCloud::Ptr LoadInputCloud(const AnalysisConfig& config, std::vector<std::string>* loaded_files)
{
  if (config.input_path.empty())
  {
    throw std::runtime_error("input_path is empty. Pass --input or update the YAML config.");
  }

  const std::vector<std::string> files = CollectInputFiles(config.input_path);
  PointCloud::Ptr cloud(new PointCloud);

  for (const auto& file : files)
  {
    std::string description;
    const PointCloud::Ptr current = LoadPointCloudFile(file, &description);
    *cloud += *current;
    if (loaded_files != nullptr)
    {
      loaded_files->push_back(description);
    }
  }

  MaybeDownsampleByStride(*cloud, config.max_points);
  return cloud;
}

std::vector<VoxelMetrics> RunVoxelMorphologyAnalysis(const PointCloud& cloud, const AnalysisConfig& config)
{
  std::unordered_map<VoxelKey, VoxelBucket, VoxelKeyHash> voxel_map;
  voxel_map.reserve(cloud.size());

  for (const auto& point : cloud.points)
  {
    const Eigen::Vector3d eigen_point(point.x, point.y, point.z);
    const VoxelKey key = MakeVoxelKey(eigen_point, config.voxel_size);
    auto& bucket = voxel_map[key];
    if (bucket.points.empty())
    {
      bucket.key = key;
      bucket.center = VoxelCenterFromKey(key, config.voxel_size);
    }
    bucket.points.push_back(eigen_point);
  }

  std::vector<VoxelMetrics> metrics;
  metrics.reserve(voxel_map.size());

  int processed_voxels = 0;
  for (const auto& item : voxel_map)
  {
    const VoxelBucket& bucket = item.second;
    if (static_cast<int>(bucket.points.size()) < config.min_points_per_voxel)
    {
      continue;
    }

    VoxelMetrics voxel_metrics;
    voxel_metrics.key = bucket.key;
    voxel_metrics.center = bucket.center;
    voxel_metrics.stats = ComputeVoxelStats(bucket);
    voxel_metrics.gaussian =
      ComputeGaussianMetrics(bucket, voxel_metrics.stats, config.gaussian_regularization);
    voxel_metrics.shell =
      ComputeShellMetrics(
        bucket,
        voxel_metrics.stats,
        config.axis_scale_quantile,
        config.axis_scale_min,
        config.shape_exponent);
    metrics.push_back(voxel_metrics);

    ++processed_voxels;
    if (config.max_voxels > 0 && processed_voxels >= config.max_voxels)
    {
      break;
    }
  }

  ComputeContextMetrics(&metrics);
  return metrics;
}

void SaveAnalysisResults(
  const std::vector<VoxelMetrics>& metrics,
  const AnalysisConfig& config,
  std::size_t input_points,
  const std::vector<std::string>& loaded_files,
  const PointCloud* input_cloud)
{
  const fs::path output_dir(config.output_dir);
  EnsureDirectory(output_dir);

  std::vector<VoxelMetrics> metrics_by_key = metrics;
  std::sort(
    metrics_by_key.begin(),
    metrics_by_key.end(),
    [](const VoxelMetrics& lhs, const VoxelMetrics& rhs)
    {
      if (lhs.key.x != rhs.key.x)
      {
        return lhs.key.x < rhs.key.x;
      }
      if (lhs.key.y != rhs.key.y)
      {
        return lhs.key.y < rhs.key.y;
      }
      return lhs.key.z < rhs.key.z;
    });

  {
    std::ofstream csv(output_dir / "voxel_metrics.csv");
    csv << "vx,vy,vz,center_x,center_y,center_z,num_points,label,"
           "eig1,eig2,eig3,linearity,planarity,scattering,anisotropy,omnivariance,"
           "gaussian_avg_mahalanobis2,gaussian_center_mahalanobis2,gaussian_avg_euclidean,gaussian_center_advantage,"
           "shell_avg_residual,shell_center_residual,shell_avg_radius,shell_center_penalty,"
           "occupied_face_count,occupied_face_ratio,opposite_face_pair_ratio,normal_variation,occupancy_asymmetry,"
           "planar_context_penalty,corner_context_bonus\n";
    for (const auto& metric : metrics_by_key)
    {
      csv << metric.key.x << ','
          << metric.key.y << ','
          << metric.key.z << ','
          << FormatDouble(metric.center.x()) << ','
          << FormatDouble(metric.center.y()) << ','
          << FormatDouble(metric.center.z()) << ','
          << metric.stats.num_points << ','
          << metric.stats.label << ','
          << FormatDouble(metric.stats.eigenvalues(0)) << ','
          << FormatDouble(metric.stats.eigenvalues(1)) << ','
          << FormatDouble(metric.stats.eigenvalues(2)) << ','
          << FormatDouble(metric.stats.linearity) << ','
          << FormatDouble(metric.stats.planarity) << ','
          << FormatDouble(metric.stats.scattering) << ','
          << FormatDouble(metric.stats.anisotropy) << ','
          << FormatDouble(metric.stats.omnivariance) << ','
          << FormatDouble(metric.gaussian.average_mahalanobis2) << ','
          << FormatDouble(metric.gaussian.voxel_center_mahalanobis2) << ','
          << FormatDouble(metric.gaussian.average_euclidean_distance) << ','
          << FormatDouble(metric.gaussian.center_advantage) << ','
          << FormatDouble(metric.shell.average_surface_residual) << ','
          << FormatDouble(metric.shell.voxel_center_surface_residual) << ','
          << FormatDouble(metric.shell.average_surface_radius) << ','
          << FormatDouble(metric.shell.center_penalty) << ','
          << metric.context.occupied_face_count << ','
          << FormatDouble(metric.context.occupied_face_ratio) << ','
          << FormatDouble(metric.context.opposite_face_pair_ratio) << ','
          << FormatDouble(metric.context.normal_variation) << ','
          << FormatDouble(metric.context.occupancy_asymmetry) << ','
          << FormatDouble(metric.context.planar_context_penalty) << ','
          << FormatDouble(metric.context.corner_context_bonus) << '\n';
    }
  }

  std::vector<VoxelMetrics> ranked = metrics;
  std::sort(
    ranked.begin(),
    ranked.end(),
    [&config](const VoxelMetrics& lhs, const VoxelMetrics& rhs)
    {
      const double lhs_score = ComputeInterestingScore(lhs, config.ranking_mode);
      const double rhs_score = ComputeInterestingScore(rhs, config.ranking_mode);
      if (lhs_score != rhs_score)
      {
        return lhs_score > rhs_score;
      }
      if (lhs.key.x != rhs.key.x)
      {
        return lhs.key.x < rhs.key.x;
      }
      if (lhs.key.y != rhs.key.y)
      {
        return lhs.key.y < rhs.key.y;
      }
      return lhs.key.z < rhs.key.z;
    });

  {
    std::ofstream csv(output_dir / "interesting_voxels.csv");
    csv << "rank,vx,vy,vz,label,num_points,gaussian_center_advantage,shell_center_penalty,"
           "ranking_adjustment,morphology_filter_penalty,context_mode_contribution,"
           "normal_variation,occupancy_asymmetry,planar_context_penalty,corner_context_bonus,total_score\n";
    const std::size_t limit =
      std::min<std::size_t>(ranked.size(), static_cast<std::size_t>(std::max(config.save_top_k, 0)));
    for (std::size_t rank = 0; rank < limit; ++rank)
    {
      const auto& metric = ranked[rank];
      csv << rank << ','
          << metric.key.x << ','
          << metric.key.y << ','
          << metric.key.z << ','
          << metric.stats.label << ','
          << metric.stats.num_points << ','
          << FormatDouble(metric.gaussian.center_advantage) << ','
          << FormatDouble(metric.shell.center_penalty) << ','
          << FormatDouble(ComputeRankingAdjustment(metric, config.ranking_mode)) << ','
          << FormatDouble(ComputeThinPlanarFilterPenalty(metric, config.ranking_mode)) << ','
          << FormatDouble(ComputeContextModeContribution(metric, config.ranking_mode)) << ','
          << FormatDouble(metric.context.normal_variation) << ','
          << FormatDouble(metric.context.occupancy_asymmetry) << ','
          << FormatDouble(metric.context.planar_context_penalty) << ','
          << FormatDouble(metric.context.corner_context_bonus) << ','
          << BuildInterestingVoxelScoreString(metric, config.ranking_mode) << '\n';
    }
  }

  {
    std::map<std::string, int> label_counts;
    std::map<std::string, int> interesting_label_counts;
    double average_gaussian_center_advantage = 0.0;
    double average_shell_center_penalty = 0.0;
    double average_filter_penalty = 0.0;
    double average_face_support = 0.0;
    double average_opposite_face_pairs = 0.0;
    double average_normal_variation = 0.0;
    double average_occupancy_asymmetry = 0.0;
    double average_planar_context_penalty = 0.0;
    double average_corner_context_bonus = 0.0;
    for (const auto& metric : metrics)
    {
      ++label_counts[metric.stats.label];
      average_gaussian_center_advantage += metric.gaussian.center_advantage;
      average_shell_center_penalty += metric.shell.center_penalty;
      average_filter_penalty += ComputeThinPlanarFilterPenalty(metric, config.ranking_mode);
      average_face_support += metric.context.occupied_face_ratio;
      average_opposite_face_pairs += metric.context.opposite_face_pair_ratio;
      average_normal_variation += metric.context.normal_variation;
      average_occupancy_asymmetry += metric.context.occupancy_asymmetry;
      average_planar_context_penalty += metric.context.planar_context_penalty;
      average_corner_context_bonus += metric.context.corner_context_bonus;
    }

    const double denom = static_cast<double>(std::max<std::size_t>(metrics.size(), 1));
    average_gaussian_center_advantage /= denom;
    average_shell_center_penalty /= denom;
    average_filter_penalty /= denom;
    average_face_support /= denom;
    average_opposite_face_pairs /= denom;
    average_normal_variation /= denom;
    average_occupancy_asymmetry /= denom;
    average_planar_context_penalty /= denom;
    average_corner_context_bonus /= denom;

    const std::size_t interesting_limit =
      std::min<std::size_t>(ranked.size(), static_cast<std::size_t>(std::max(config.save_top_k, 0)));
    for (std::size_t rank = 0; rank < interesting_limit; ++rank)
    {
      ++interesting_label_counts[ranked[rank].stats.label];
    }

    std::ofstream summary(output_dir / "summary.txt");
    summary << "input_points: " << input_points << '\n';
    summary << "analyzed_voxels: " << metrics.size() << '\n';
    summary << "input_mode: " << config.input_mode << '\n';
    summary << "voxel_size: " << FormatDouble(config.voxel_size) << '\n';
    summary << "min_points_per_voxel: " << config.min_points_per_voxel << '\n';
    summary << "shape_exponent: " << FormatDouble(config.shape_exponent) << '\n';
    summary << "axis_scale_quantile: " << FormatDouble(config.axis_scale_quantile) << '\n';
    summary << "ranking_mode: " << config.ranking_mode << '\n';
    summary << "interesting_top_k: " << interesting_limit << '\n';
    summary << "average_gaussian_center_advantage: " << FormatDouble(average_gaussian_center_advantage) << '\n';
    summary << "average_shell_center_penalty: " << FormatDouble(average_shell_center_penalty) << '\n';
    summary << "average_morphology_filter_penalty: " << FormatDouble(average_filter_penalty) << '\n';
    summary << "average_occupied_face_ratio: " << FormatDouble(average_face_support) << '\n';
    summary << "average_opposite_face_pair_ratio: " << FormatDouble(average_opposite_face_pairs) << '\n';
    summary << "average_normal_variation: " << FormatDouble(average_normal_variation) << '\n';
    summary << "average_occupancy_asymmetry: " << FormatDouble(average_occupancy_asymmetry) << '\n';
    summary << "average_planar_context_penalty: " << FormatDouble(average_planar_context_penalty) << '\n';
    summary << "average_corner_context_bonus: " << FormatDouble(average_corner_context_bonus) << '\n';
    summary << "label_counts:\n";
    for (const auto& label_count : label_counts)
    {
      summary << "  " << label_count.first << ": " << label_count.second << '\n';
    }
    summary << "interesting_label_counts:\n";
    for (const auto& label_count : interesting_label_counts)
    {
      summary << "  " << label_count.first << ": " << label_count.second << '\n';
    }
  }

  {
    std::ofstream input_log(output_dir / "loaded_files.txt");
    for (const auto& file : loaded_files)
    {
      input_log << file << '\n';
    }
  }

  if (input_cloud != nullptr)
  {
    ExportInterestingVoxelClouds(ranked, config, *input_cloud, output_dir);
  }
}

}  // namespace ges_voxel_mapping
