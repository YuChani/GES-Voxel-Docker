#include "ges_voxel_mapping/direct_primitive_validation.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace fs = std::filesystem;

namespace ges_voxel_mapping
{
namespace
{

constexpr double kEpsilon = 1e-9;

struct PrimitiveVoxelBucket
{
  VoxelKey key;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> points;
};

struct PrimitiveStats
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
  bool degenerate = false;
};

struct GaussianFit
{
  double average_mahalanobis2 = 0.0;
  double voxel_center_mahalanobis2 = 0.0;
  double average_center_distance = 0.0;
  double condition_ratio = 0.0;
};

struct SurfaceShellFit
{
  double normal_rms = 0.0;
  double average_shell_residual = 0.0;
  double voxel_center_shell_residual = 0.0;
  double average_radius = 0.0;
  Eigen::Vector3d axis_scales = Eigen::Vector3d::Zero();
  double axis_condition = 0.0;
};

std::string FormatDouble(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  return stream.str();
}

std::string VoxelId(const VoxelKey& key)
{
  return std::to_string(key.x) + "_" + std::to_string(key.y) + "_" + std::to_string(key.z);
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
  const double half = voxel_size * 0.5;
  return Eigen::Vector3d(
    static_cast<double>(key.x) * voxel_size + half,
    static_cast<double>(key.y) * voxel_size + half,
    static_cast<double>(key.z) * voxel_size + half);
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

double ComputeLpRadius(
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

PrimitiveStats ComputePrimitiveStats(const PrimitiveVoxelBucket& voxel)
{
  PrimitiveStats stats;
  stats.num_points = voxel.points.size();
  if (voxel.points.empty())
  {
    stats.degenerate = true;
    stats.label = "degenerate";
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
    stats.degenerate = true;
    stats.label = "eigensolver_failed";
    return stats;
  }

  const Eigen::Vector3d ascending_values = solver.eigenvalues();
  const Eigen::Matrix3d ascending_vectors = solver.eigenvectors();
  for (int axis = 0; axis < 3; ++axis)
  {
    stats.eigenvalues(axis) = std::max(ascending_values(2 - axis), 0.0);
    stats.eigenvectors.col(axis) = ascending_vectors.col(2 - axis);
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
    stats.degenerate = true;
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

GaussianFit FitGaussianPrimitive(
  const PrimitiveVoxelBucket& voxel,
  const PrimitiveStats& stats,
  double regularization)
{
  GaussianFit fit;

  Eigen::Matrix3d covariance_reg = stats.covariance;
  covariance_reg.diagonal().array() += regularization;
  const Eigen::Matrix3d covariance_inv = covariance_reg.inverse();

  for (const auto& point : voxel.points)
  {
    fit.average_mahalanobis2 += MahalanobisSquared(point, stats.mean, covariance_inv);
    fit.average_center_distance += (point - stats.mean).norm();
  }

  const double count = static_cast<double>(std::max<std::size_t>(voxel.points.size(), 1));
  fit.average_mahalanobis2 /= count;
  fit.average_center_distance /= count;
  fit.voxel_center_mahalanobis2 = MahalanobisSquared(voxel.center, stats.mean, covariance_inv);
  fit.condition_ratio =
    std::sqrt(std::max(stats.eigenvalues(2), 0.0)) / std::sqrt(std::max(stats.eigenvalues(0), kEpsilon));
  return fit;
}

SurfaceShellFit FitSurfaceShellPrimitive(
  const PrimitiveVoxelBucket& voxel,
  const PrimitiveStats& stats,
  double axis_scale_quantile,
  double axis_scale_min,
  double shape_exponent)
{
  SurfaceShellFit fit;

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
    fit.normal_rms += local(2) * local(2);
    for (int axis = 0; axis < 3; ++axis)
    {
      axis_abs[axis].push_back(std::abs(local(axis)));
    }
  }

  const double count = static_cast<double>(std::max<std::size_t>(voxel.points.size(), 1));
  fit.normal_rms = std::sqrt(fit.normal_rms / count);

  for (int axis = 0; axis < 3; ++axis)
  {
    fit.axis_scales(axis) = std::max(ComputeQuantile(axis_abs[axis], axis_scale_quantile), axis_scale_min);
  }

  for (const auto& local_point : local_points)
  {
    const double radius = ComputeLpRadius(local_point, fit.axis_scales, shape_exponent);
    fit.average_radius += radius;
    fit.average_shell_residual += std::abs(radius - 1.0);
  }
  fit.average_radius /= count;
  fit.average_shell_residual /= count;

  const Eigen::Vector3d center_local = stats.eigenvectors.transpose() * (voxel.center - stats.mean);
  fit.voxel_center_shell_residual = std::abs(ComputeLpRadius(center_local, fit.axis_scales, shape_exponent) - 1.0);
  fit.axis_condition = fit.axis_scales(2) / std::max(fit.axis_scales(0), kEpsilon);
  return fit;
}

double ComputeCornerSelectionScore(const PrimitiveVoxelComparison& voxel)
{
  return std::min(voxel.linearity, voxel.planarity) + 0.5 * voxel.scattering;
}

double ComputePlanarSelectionScore(const PrimitiveVoxelComparison& voxel)
{
  return voxel.planarity + 0.25 * voxel.anisotropy;
}

double ComputeBoundaryProxySelectionScore(const PrimitiveVoxelComparison& voxel)
{
  return voxel.linearity + 0.2 * voxel.anisotropy;
}

struct CandidateRef
{
  std::size_t index = 0;
  double score = 0.0;
  std::size_t point_count = 0;
};

void SortCandidates(std::vector<CandidateRef>* candidates)
{
  std::sort(
    candidates->begin(),
    candidates->end(),
    [](const CandidateRef& lhs, const CandidateRef& rhs)
    {
      if (lhs.score != rhs.score)
      {
        return lhs.score > rhs.score;
      }
      if (lhs.point_count != rhs.point_count)
      {
        return lhs.point_count > rhs.point_count;
      }
      return lhs.index < rhs.index;
    });
}

std::vector<std::size_t> SelectCandidateIndices(
  const std::vector<PrimitiveVoxelComparison>& all_voxels,
  const PrimitiveValidationConfig& config)
{
  if (config.max_voxels <= 0)
  {
    return {};
  }

  std::vector<CandidateRef> planar_candidates;
  std::vector<CandidateRef> corner_candidates;
  std::vector<CandidateRef> boundary_candidates;

  for (std::size_t index = 0; index < all_voxels.size(); ++index)
  {
    const auto& voxel = all_voxels[index];
    if (voxel.degenerate)
    {
      continue;
    }

    if (voxel.base_label == "planar")
    {
      planar_candidates.push_back(CandidateRef{
        index,
        ComputePlanarSelectionScore(voxel),
        voxel.point_count});
    }
    else if (voxel.base_label == "corner_like")
    {
      corner_candidates.push_back(CandidateRef{
        index,
        ComputeCornerSelectionScore(voxel),
        voxel.point_count});
    }
    else if (voxel.base_label == "linear")
    {
      boundary_candidates.push_back(CandidateRef{
        index,
        ComputeBoundaryProxySelectionScore(voxel),
        voxel.point_count});
    }
  }

  SortCandidates(&planar_candidates);
  SortCandidates(&corner_candidates);
  SortCandidates(&boundary_candidates);

  std::vector<std::size_t> selected;
  selected.reserve(static_cast<std::size_t>(config.max_voxels));
  std::set<std::size_t> used;

  const auto take_from_category =
    [&](const std::vector<CandidateRef>& category_candidates)
    {
      int taken = 0;
      for (const auto& candidate : category_candidates)
      {
        if (static_cast<int>(selected.size()) >= config.max_voxels || taken >= config.per_category_limit)
        {
          break;
        }
        if (!used.insert(candidate.index).second)
        {
          continue;
        }
        selected.push_back(candidate.index);
        ++taken;
      }
    };

  take_from_category(planar_candidates);
  take_from_category(corner_candidates);
  take_from_category(boundary_candidates);

  if (static_cast<int>(selected.size()) < config.max_voxels)
  {
    std::vector<CandidateRef> fallback;
    fallback.reserve(all_voxels.size());
    for (std::size_t index = 0; index < all_voxels.size(); ++index)
    {
      if (used.count(index) != 0 || all_voxels[index].degenerate)
      {
        continue;
      }
      fallback.push_back(CandidateRef{index, static_cast<double>(all_voxels[index].point_count), all_voxels[index].point_count});
    }
    SortCandidates(&fallback);
    for (const auto& candidate : fallback)
    {
      if (static_cast<int>(selected.size()) >= config.max_voxels)
      {
        break;
      }
      selected.push_back(candidate.index);
    }
  }

  return selected;
}

std::string SelectionNote(const PrimitiveVoxelComparison& voxel)
{
  if (voxel.selection_tag == "planar_candidate")
  {
    return "base_label=planar, high planarity representative";
  }
  if (voxel.selection_tag == "corner_candidate")
  {
    return "base_label=corner_like, corner/crease representative";
  }
  if (voxel.selection_tag == "boundary_candidate_proxy")
  {
    return "base_label=linear, boundary/mixed proxy used only for Stage-1 selection";
  }
  return "fallback representative selected by point count";
}

void WriteLoadedFiles(const fs::path& output_dir, const std::vector<std::string>& loaded_files)
{
  std::ofstream stream(output_dir / "loaded_files.txt");
  for (const auto& line : loaded_files)
  {
    stream << line << '\n';
  }
}

void WriteSelectedVoxelsCsv(
  const fs::path& output_dir,
  const std::vector<PrimitiveVoxelComparison>& comparisons)
{
  std::ofstream stream(output_dir / "selected_voxels.csv");
  stream
    << "voxel_id,key_x,key_y,key_z,selection_tag,base_label,point_count,selection_note\n";
  for (const auto& comparison : comparisons)
  {
    stream
      << VoxelId(comparison.key) << ','
      << comparison.key.x << ','
      << comparison.key.y << ','
      << comparison.key.z << ','
      << comparison.selection_tag << ','
      << comparison.base_label << ','
      << comparison.point_count << ','
      << comparison.selection_note << '\n';
  }
}

void WriteComparisonCsv(
  const fs::path& output_dir,
  const std::vector<PrimitiveVoxelComparison>& comparisons)
{
  std::ofstream stream(output_dir / "voxel_comparison.csv");
  stream
    << "voxel_id,key_x,key_y,key_z,selection_tag,base_label,point_count,"
    << "linearity,planarity,scattering,anisotropy,omnivariance,"
    << "gaussian_average_mahalanobis2,gaussian_voxel_center_mahalanobis2,"
    << "gaussian_average_center_distance,gaussian_condition_ratio,"
    << "surface_normal_rms,shell_average_residual,shell_voxel_center_residual,"
    << "shell_average_radius,shell_axis_scale_major,shell_axis_scale_middle,"
    << "shell_axis_scale_minor,shell_axis_condition,sparsity_indicator,degenerate\n";

  for (const auto& comparison : comparisons)
  {
    stream
      << VoxelId(comparison.key) << ','
      << comparison.key.x << ','
      << comparison.key.y << ','
      << comparison.key.z << ','
      << comparison.selection_tag << ','
      << comparison.base_label << ','
      << comparison.point_count << ','
      << FormatDouble(comparison.linearity) << ','
      << FormatDouble(comparison.planarity) << ','
      << FormatDouble(comparison.scattering) << ','
      << FormatDouble(comparison.anisotropy) << ','
      << FormatDouble(comparison.omnivariance) << ','
      << FormatDouble(comparison.gaussian_average_mahalanobis2) << ','
      << FormatDouble(comparison.gaussian_voxel_center_mahalanobis2) << ','
      << FormatDouble(comparison.gaussian_average_center_distance) << ','
      << FormatDouble(comparison.gaussian_condition_ratio) << ','
      << FormatDouble(comparison.surface_normal_rms) << ','
      << FormatDouble(comparison.shell_average_residual) << ','
      << FormatDouble(comparison.shell_voxel_center_residual) << ','
      << FormatDouble(comparison.shell_average_radius) << ','
      << FormatDouble(comparison.shell_axis_scales(0)) << ','
      << FormatDouble(comparison.shell_axis_scales(1)) << ','
      << FormatDouble(comparison.shell_axis_scales(2)) << ','
      << FormatDouble(comparison.shell_axis_condition) << ','
      << FormatDouble(comparison.sparsity_indicator) << ','
      << (comparison.degenerate ? "true" : "false") << '\n';
  }
}

void WriteSummary(
  const fs::path& output_dir,
  const PrimitiveValidationConfig& config,
  std::size_t input_points,
  const std::vector<std::string>& loaded_files,
  const std::vector<PrimitiveVoxelComparison>& comparisons)
{
  std::ofstream stream(output_dir / "summary.md");
  stream << "# Direct Primitive Validation Stage-1\n\n";
  stream << "- input_path: `" << config.input_path << "`\n";
  stream << "- output_dir: `" << config.output_dir << "`\n";
  stream << "- input_points: `" << input_points << "`\n";
  stream << "- loaded_files: `" << loaded_files.size() << "`\n";
  stream << "- voxel_size: `" << config.voxel_size << "`\n";
  stream << "- min_points_per_voxel: `" << config.min_points_per_voxel << "`\n";
  stream << "- max_voxels: `" << config.max_voxels << "`\n";
  stream << "- per_category_limit: `" << config.per_category_limit << "`\n";
  stream << "- selection_mode: `" << config.selection_mode << "`\n";
  stream << "- gaussian_model: `mean/covariance baseline`\n";
  stream << "- surface_model: `PCA local frame + quantile axis scales + L_p shell scaffold`\n";
  stream << "- note: Stage-1의 boundary/mixed category는 robust semantic label이 아니라 `linear` voxel proxy를 사용한다.\n\n";

  struct Aggregate
  {
    int count = 0;
    double gaussian_average_mahalanobis2 = 0.0;
    double gaussian_average_center_distance = 0.0;
    double surface_normal_rms = 0.0;
    double shell_average_residual = 0.0;
  };

  std::unordered_map<std::string, Aggregate> aggregates;
  for (const auto& comparison : comparisons)
  {
    auto& aggregate = aggregates[comparison.selection_tag];
    ++aggregate.count;
    aggregate.gaussian_average_mahalanobis2 += comparison.gaussian_average_mahalanobis2;
    aggregate.gaussian_average_center_distance += comparison.gaussian_average_center_distance;
    aggregate.surface_normal_rms += comparison.surface_normal_rms;
    aggregate.shell_average_residual += comparison.shell_average_residual;
  }

  stream << "## Category Means\n\n";
  stream << "| selection_tag | count | gaussian_avg_mahalanobis2 | gaussian_avg_center_distance | surface_normal_rms | shell_average_residual |\n";
  stream << "| --- | ---: | ---: | ---: | ---: | ---: |\n";

  std::vector<std::string> ordered_tags;
  ordered_tags.reserve(aggregates.size());
  for (const auto& item : aggregates)
  {
    ordered_tags.push_back(item.first);
  }
  std::sort(ordered_tags.begin(), ordered_tags.end());
  for (const auto& tag : ordered_tags)
  {
    const auto& aggregate = aggregates.at(tag);
    const double denom = static_cast<double>(std::max(aggregate.count, 1));
    stream
      << "| " << tag
      << " | " << aggregate.count
      << " | " << FormatDouble(aggregate.gaussian_average_mahalanobis2 / denom)
      << " | " << FormatDouble(aggregate.gaussian_average_center_distance / denom)
      << " | " << FormatDouble(aggregate.surface_normal_rms / denom)
      << " | " << FormatDouble(aggregate.shell_average_residual / denom)
      << " |\n";
  }

  stream << "\n## Selected Voxels\n\n";
  for (const auto& comparison : comparisons)
  {
    stream
      << "- `" << VoxelId(comparison.key) << "`"
      << " | " << comparison.selection_tag
      << " | base_label=`" << comparison.base_label << "`"
      << " | points=`" << comparison.point_count << "`"
      << " | gaussian_avg_mahalanobis2=`" << FormatDouble(comparison.gaussian_average_mahalanobis2) << "`"
      << " | shell_average_residual=`" << FormatDouble(comparison.shell_average_residual) << "`"
      << " | surface_normal_rms=`" << FormatDouble(comparison.surface_normal_rms) << "`\n";
  }

  stream << "\n## Interpretation Guardrails\n\n";
  stream << "- 이 출력은 Stage-1 scaffold다. exact GES/GND fitting이나 registration 성능을 아직 주장하지 않는다.\n";
  stream << "- gaussian residual과 shell residual은 scale이 다를 수 있으므로, 현재 단계에서는 직접적인 승패 선언보다 category별 경향 확인에 사용한다.\n";
  stream << "- 다음 단계는 voxel-level residual normalization 검토와 local registration residual 비교다.\n";
}

}  // namespace

PointCloud::Ptr LoadPrimitiveValidationCloud(
  const PrimitiveValidationConfig& config,
  std::vector<std::string>* loaded_files)
{
  AnalysisConfig load_config;
  load_config.input_path = config.input_path;
  load_config.max_points = config.max_points;
  return LoadInputCloud(load_config, loaded_files);
}

std::vector<PrimitiveVoxelComparison> RunDirectPrimitiveValidation(
  const PointCloud& cloud,
  const PrimitiveValidationConfig& config)
{
  std::unordered_map<VoxelKey, PrimitiveVoxelBucket, VoxelKeyHash> voxel_map;
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

  std::vector<PrimitiveVoxelComparison> all_voxels;
  all_voxels.reserve(voxel_map.size());

  for (const auto& item : voxel_map)
  {
    const PrimitiveVoxelBucket& bucket = item.second;
    if (static_cast<int>(bucket.points.size()) < config.min_points_per_voxel)
    {
      continue;
    }

    const PrimitiveStats stats = ComputePrimitiveStats(bucket);
    const GaussianFit gaussian = FitGaussianPrimitive(bucket, stats, config.gaussian_regularization);
    const SurfaceShellFit shell = FitSurfaceShellPrimitive(
      bucket,
      stats,
      config.shell_axis_scale_quantile,
      config.shell_axis_scale_min,
      config.shell_shape_exponent);

    PrimitiveVoxelComparison comparison;
    comparison.key = bucket.key;
    comparison.voxel_center = bucket.center;
    comparison.mean = stats.mean;
    comparison.eigenvalues = stats.eigenvalues;
    comparison.point_count = stats.num_points;
    comparison.base_label = stats.label;
    comparison.linearity = stats.linearity;
    comparison.planarity = stats.planarity;
    comparison.scattering = stats.scattering;
    comparison.anisotropy = stats.anisotropy;
    comparison.omnivariance = stats.omnivariance;
    comparison.gaussian_average_mahalanobis2 = gaussian.average_mahalanobis2;
    comparison.gaussian_voxel_center_mahalanobis2 = gaussian.voxel_center_mahalanobis2;
    comparison.gaussian_average_center_distance = gaussian.average_center_distance;
    comparison.gaussian_condition_ratio = gaussian.condition_ratio;
    comparison.surface_normal_rms = shell.normal_rms;
    comparison.shell_average_residual = shell.average_shell_residual;
    comparison.shell_voxel_center_residual = shell.voxel_center_shell_residual;
    comparison.shell_average_radius = shell.average_radius;
    comparison.shell_axis_scales = shell.axis_scales;
    comparison.shell_axis_condition = shell.axis_condition;
    comparison.sparsity_indicator =
      1.0 / std::sqrt(static_cast<double>(std::max<std::size_t>(comparison.point_count, 1)));
    comparison.degenerate = stats.degenerate;
    all_voxels.push_back(comparison);
  }

  const std::vector<std::size_t> selected_indices = SelectCandidateIndices(all_voxels, config);
  std::vector<PrimitiveVoxelComparison> selected;
  selected.reserve(selected_indices.size());

  for (const std::size_t index : selected_indices)
  {
    PrimitiveVoxelComparison comparison = all_voxels[index];
    if (comparison.base_label == "planar")
    {
      comparison.selection_tag = "planar_candidate";
    }
    else if (comparison.base_label == "corner_like")
    {
      comparison.selection_tag = "corner_candidate";
    }
    else if (comparison.base_label == "linear")
    {
      comparison.selection_tag = "boundary_candidate_proxy";
    }
    else
    {
      comparison.selection_tag = "fallback_candidate";
    }
    comparison.selection_note = SelectionNote(comparison);
    selected.push_back(comparison);
  }

  std::sort(
    selected.begin(),
    selected.end(),
    [](const PrimitiveVoxelComparison& lhs, const PrimitiveVoxelComparison& rhs)
    {
      if (lhs.selection_tag != rhs.selection_tag)
      {
        return lhs.selection_tag < rhs.selection_tag;
      }
      if (lhs.point_count != rhs.point_count)
      {
        return lhs.point_count > rhs.point_count;
      }
      return VoxelId(lhs.key) < VoxelId(rhs.key);
    });

  return selected;
}

void SaveDirectPrimitiveValidationResults(
  const std::vector<PrimitiveVoxelComparison>& comparisons,
  const PrimitiveValidationConfig& config,
  std::size_t input_points,
  const std::vector<std::string>& loaded_files)
{
  const fs::path output_dir(config.output_dir);
  fs::create_directories(output_dir);

  WriteLoadedFiles(output_dir, loaded_files);
  WriteSelectedVoxelsCsv(output_dir, comparisons);
  WriteComparisonCsv(output_dir, comparisons);
  WriteSummary(output_dir, config, input_points, loaded_files, comparisons);
}

}  // namespace ges_voxel_mapping
