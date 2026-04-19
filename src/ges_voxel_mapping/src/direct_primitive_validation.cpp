#include "ges_voxel_mapping/direct_primitive_validation.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
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
constexpr double kPi = 3.14159265358979323846;

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
  Eigen::Matrix3d covariance_inverse = Eigen::Matrix3d::Identity();
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

struct PrimitiveModelBundle
{
  PrimitiveVoxelBucket bucket;
  PrimitiveStats stats;
  GaussianFit gaussian;
  SurfaceShellFit shell;
  PrimitiveVoxelComparison comparison;
};

struct ReferencePrimitive
{
  PrimitiveStats stats;
  GaussianFit gaussian;
  SurfaceShellFit shell;
};

struct PerturbationSpec
{
  std::string name;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation_axis = Eigen::Vector3d::UnitZ();
  double rotation_radians = 0.0;
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

bool VoxelKeyLess(const VoxelKey& lhs, const VoxelKey& rhs)
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
  fit.covariance_inverse = covariance_reg.inverse();

  for (const auto& point : voxel.points)
  {
    fit.average_mahalanobis2 += MahalanobisSquared(point, stats.mean, fit.covariance_inverse);
    fit.average_center_distance += (point - stats.mean).norm();
  }

  const double count = static_cast<double>(std::max<std::size_t>(voxel.points.size(), 1));
  fit.average_mahalanobis2 /= count;
  fit.average_center_distance /= count;
  fit.voxel_center_mahalanobis2 = MahalanobisSquared(voxel.center, stats.mean, fit.covariance_inverse);
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
      planar_candidates.push_back(CandidateRef{index, ComputePlanarSelectionScore(voxel), voxel.point_count});
    }
    else if (voxel.base_label == "corner_like")
    {
      corner_candidates.push_back(CandidateRef{index, ComputeCornerSelectionScore(voxel), voxel.point_count});
    }
    else if (voxel.base_label == "linear")
    {
      boundary_candidates.push_back(CandidateRef{index, ComputeBoundaryProxySelectionScore(voxel), voxel.point_count});
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
      fallback.push_back(CandidateRef{
        index,
        static_cast<double>(all_voxels[index].point_count),
        all_voxels[index].point_count});
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

std::string SelectionTagFromBaseLabel(const std::string& base_label)
{
  if (base_label == "planar")
  {
    return "planar_candidate";
  }
  if (base_label == "corner_like")
  {
    return "corner_candidate";
  }
  if (base_label == "linear")
  {
    return "boundary_candidate_proxy";
  }
  return "fallback_candidate";
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
    return "base_label=linear, boundary/mixed proxy used only for Stage-2 quickcheck";
  }
  return "fallback representative selected by point count";
}

double ComputeCharacteristicSpreadScale(const PrimitiveStats& stats)
{
  const double s1 = std::sqrt(std::max(stats.eigenvalues(0), kEpsilon));
  const double s2 = std::sqrt(std::max(stats.eigenvalues(1), kEpsilon));
  const double s3 = std::sqrt(std::max(stats.eigenvalues(2), kEpsilon));
  return std::max((s1 + s2 + s3) / 3.0, kEpsilon);
}

double ComputeShellScaleMean(const SurfaceShellFit& shell)
{
  return std::max(
    (shell.axis_scales(0) + shell.axis_scales(1) + shell.axis_scales(2)) / 3.0,
    kEpsilon);
}

void FillNormalizedMetrics(
  PrimitiveVoxelComparison* comparison,
  const PrimitiveStats& stats,
  const SurfaceShellFit& shell,
  double voxel_size)
{
  comparison->characteristic_spread_scale = ComputeCharacteristicSpreadScale(stats);
  comparison->shell_scale_mean = ComputeShellScaleMean(shell);
  comparison->shell_geometric_average_residual =
    comparison->shell_average_residual * comparison->shell_scale_mean;
  comparison->shell_geometric_center_residual =
    comparison->shell_voxel_center_residual * comparison->shell_scale_mean;

  comparison->gaussian_normalized_residual_by_voxel =
    comparison->gaussian_average_center_distance / std::max(voxel_size, kEpsilon);
  comparison->shell_normalized_residual_by_voxel =
    comparison->shell_geometric_average_residual / std::max(voxel_size, kEpsilon);
  comparison->gaussian_normalized_residual_by_spread =
    comparison->gaussian_average_center_distance / comparison->characteristic_spread_scale;
  comparison->shell_normalized_residual_by_spread =
    comparison->shell_geometric_average_residual / comparison->characteristic_spread_scale;
  comparison->normalized_residual_gap =
    comparison->gaussian_normalized_residual_by_spread -
    comparison->shell_normalized_residual_by_spread;
}

std::vector<Eigen::Vector3d> CollectNeighborhoodPoints(
  const VoxelKey& center_key,
  int radius_voxels,
  const std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash>& lookup,
  const std::vector<PrimitiveVoxelBucket>& buckets)
{
  std::vector<Eigen::Vector3d> points;
  for (int dx = -radius_voxels; dx <= radius_voxels; ++dx)
  {
    for (int dy = -radius_voxels; dy <= radius_voxels; ++dy)
    {
      for (int dz = -radius_voxels; dz <= radius_voxels; ++dz)
      {
        const VoxelKey key{center_key.x + dx, center_key.y + dy, center_key.z + dz};
        const auto it = lookup.find(key);
        if (it == lookup.end())
        {
          continue;
        }
        const auto& bucket = buckets[it->second];
        points.insert(points.end(), bucket.points.begin(), bucket.points.end());
      }
    }
  }
  return points;
}

PrimitiveVoxelBucket MakeBucketFromPoints(
  const std::vector<Eigen::Vector3d>& points,
  const VoxelKey& key,
  const Eigen::Vector3d& center)
{
  PrimitiveVoxelBucket bucket;
  bucket.key = key;
  bucket.center = center;
  bucket.points = points;
  return bucket;
}

ReferencePrimitive BuildReferencePrimitive(
  const std::vector<Eigen::Vector3d>& reference_points,
  const PrimitiveVoxelComparison& comparison,
  const PrimitiveValidationConfig& config)
{
  const PrimitiveVoxelBucket bucket = MakeBucketFromPoints(reference_points, comparison.key, comparison.voxel_center);
  ReferencePrimitive reference;
  reference.stats = ComputePrimitiveStats(bucket);
  reference.gaussian = FitGaussianPrimitive(bucket, reference.stats, config.gaussian_regularization);
  reference.shell = FitSurfaceShellPrimitive(
    bucket,
    reference.stats,
    config.shell_axis_scale_quantile,
    config.shell_axis_scale_min,
    config.shell_shape_exponent);
  return reference;
}

std::vector<Eigen::Vector3d> TransformPoints(
  const std::vector<Eigen::Vector3d>& points,
  const Eigen::Vector3d& pivot,
  const PerturbationSpec& perturbation)
{
  std::vector<Eigen::Vector3d> transformed;
  transformed.reserve(points.size());

  const bool use_rotation =
    perturbation.rotation_radians > 0.0 &&
    perturbation.rotation_axis.norm() > kEpsilon;
  const Eigen::AngleAxisd rotation(
    perturbation.rotation_radians,
    use_rotation ? perturbation.rotation_axis.normalized() : Eigen::Vector3d::UnitX());

  for (const auto& point : points)
  {
    Eigen::Vector3d current = point;
    if (use_rotation)
    {
      current = pivot + rotation * (current - pivot);
    }
    current += perturbation.translation;
    transformed.push_back(current);
  }
  return transformed;
}

double EvaluateGaussianQuickScore(
  const std::vector<Eigen::Vector3d>& points,
  const ReferencePrimitive& reference)
{
  double accum = 0.0;
  for (const auto& point : points)
  {
    accum += std::sqrt(std::max(
      MahalanobisSquared(point, reference.stats.mean, reference.gaussian.covariance_inverse),
      0.0)) / std::sqrt(3.0);
  }
  return accum / static_cast<double>(std::max<std::size_t>(points.size(), 1));
}

double EvaluateShellQuickScore(
  const std::vector<Eigen::Vector3d>& points,
  const ReferencePrimitive& reference,
  double shape_exponent)
{
  double accum = 0.0;
  for (const auto& point : points)
  {
    const Eigen::Vector3d local =
      reference.stats.eigenvectors.transpose() * (point - reference.stats.mean);
    const double radius = ComputeLpRadius(local, reference.shell.axis_scales, shape_exponent);
    accum += std::abs(radius - 1.0);
  }
  return accum / static_cast<double>(std::max<std::size_t>(points.size(), 1));
}

std::vector<PerturbationSpec> BuildPerturbations(
  const ReferencePrimitive& reference,
  const PrimitiveValidationConfig& config)
{
  const double translation_step = config.registration_translation_step_ratio * config.voxel_size;
  const double rotation_radians = config.registration_rotation_degrees * kPi / 180.0;
  const Eigen::Vector3d major_axis = reference.stats.eigenvectors.col(0).normalized();
  const Eigen::Vector3d middle_axis = reference.stats.eigenvectors.col(1).normalized();
  const Eigen::Vector3d normal_axis = reference.stats.eigenvectors.col(2).normalized();

  return {
    PerturbationSpec{"translate_normal", normal_axis * translation_step, Eigen::Vector3d::UnitZ(), 0.0},
    PerturbationSpec{"translate_major", major_axis * translation_step, Eigen::Vector3d::UnitZ(), 0.0},
    PerturbationSpec{"rotate_middle", Eigen::Vector3d::Zero(), middle_axis, rotation_radians}};
}

void AppendRegistrationQuickchecks(
  std::vector<PrimitiveRegistrationQuickcheck>* output,
  PrimitiveVoxelComparison* comparison,
  const std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash>& lookup,
  const std::vector<PrimitiveVoxelBucket>& buckets,
  const PrimitiveValidationConfig& config)
{
  std::vector<Eigen::Vector3d> local_points =
    CollectNeighborhoodPoints(comparison->key, config.registration_neighborhood_voxels, lookup, buckets);
  comparison->local_neighborhood_point_count = static_cast<int>(local_points.size());
  if (static_cast<int>(local_points.size()) < config.registration_min_points)
  {
    return;
  }

  std::vector<Eigen::Vector3d> reference_points;
  std::vector<Eigen::Vector3d> source_points;
  reference_points.reserve((local_points.size() + 1) / 2);
  source_points.reserve(local_points.size() / 2);

  for (std::size_t index = 0; index < local_points.size(); ++index)
  {
    if (index % 2 == 0)
    {
      reference_points.push_back(local_points[index]);
    }
    else
    {
      source_points.push_back(local_points[index]);
    }
  }

  if (
    static_cast<int>(reference_points.size()) < config.registration_min_points / 2 ||
    static_cast<int>(source_points.size()) < config.registration_min_points / 2)
  {
    return;
  }

  const ReferencePrimitive reference = BuildReferencePrimitive(reference_points, *comparison, config);
  const double gaussian_nominal = EvaluateGaussianQuickScore(source_points, reference);
  const double shell_nominal = EvaluateShellQuickScore(source_points, reference, config.shell_shape_exponent);
  const std::vector<PerturbationSpec> perturbations = BuildPerturbations(reference, config);

  for (const auto& perturbation : perturbations)
  {
    const std::vector<Eigen::Vector3d> transformed =
      TransformPoints(source_points, reference.stats.mean, perturbation);
    const double gaussian_perturbed = EvaluateGaussianQuickScore(transformed, reference);
    const double shell_perturbed = EvaluateShellQuickScore(transformed, reference, config.shell_shape_exponent);

    PrimitiveRegistrationQuickcheck quickcheck;
    quickcheck.key = comparison->key;
    quickcheck.voxel_id = VoxelId(comparison->key);
    quickcheck.selection_tag = comparison->selection_tag;
    quickcheck.base_label = comparison->base_label;
    quickcheck.perturbation = perturbation.name;
    quickcheck.neighborhood_point_count = static_cast<int>(local_points.size());
    quickcheck.reference_point_count = static_cast<int>(reference_points.size());
    quickcheck.source_point_count = static_cast<int>(source_points.size());
    quickcheck.perturbation_translation_norm = perturbation.translation.norm();
    quickcheck.perturbation_rotation_degrees = perturbation.rotation_radians * 180.0 / kPi;
    quickcheck.gaussian_nominal_score = gaussian_nominal;
    quickcheck.gaussian_perturbed_score = gaussian_perturbed;
    quickcheck.gaussian_delta = gaussian_perturbed - gaussian_nominal;
    quickcheck.shell_nominal_score = shell_nominal;
    quickcheck.shell_perturbed_score = shell_perturbed;
    quickcheck.shell_delta = shell_perturbed - shell_nominal;
    quickcheck.delta_advantage = quickcheck.shell_delta - quickcheck.gaussian_delta;
    quickcheck.shell_better_discrimination = quickcheck.delta_advantage > 0.0;
    output->push_back(quickcheck);
  }
}

void WriteLoadedFiles(const fs::path& output_dir, const std::vector<std::string>& loaded_files)
{
  std::ofstream stream(output_dir / "loaded_files.txt");
  for (const auto& line : loaded_files)
  {
    stream << line << '\n';
  }
}

void WriteSelectedCasesCsv(
  const fs::path& output_dir,
  const std::vector<PrimitiveVoxelComparison>& comparisons)
{
  std::ofstream stream(output_dir / "selected_cases.csv");
  stream
    << "voxel_id,key_x,key_y,key_z,selection_tag,base_label,point_count,local_neighborhood_point_count,"
    << "selection_note,gaussian_normalized_residual_by_spread,shell_normalized_residual_by_spread,"
    << "normalized_residual_gap\n";
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
      << comparison.local_neighborhood_point_count << ','
      << '"' << comparison.selection_note << '"' << ','
      << FormatDouble(comparison.gaussian_normalized_residual_by_spread) << ','
      << FormatDouble(comparison.shell_normalized_residual_by_spread) << ','
      << FormatDouble(comparison.normalized_residual_gap) << '\n';
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
    << "shell_axis_scale_minor,shell_axis_condition,sparsity_indicator,"
    << "characteristic_spread_scale,shell_scale_mean,shell_geometric_average_residual,"
    << "shell_geometric_center_residual,gaussian_normalized_residual_by_voxel,"
    << "shell_normalized_residual_by_voxel,gaussian_normalized_residual_by_spread,"
    << "shell_normalized_residual_by_spread,normalized_residual_gap,"
    << "local_neighborhood_point_count,degenerate\n";

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
      << FormatDouble(comparison.characteristic_spread_scale) << ','
      << FormatDouble(comparison.shell_scale_mean) << ','
      << FormatDouble(comparison.shell_geometric_average_residual) << ','
      << FormatDouble(comparison.shell_geometric_center_residual) << ','
      << FormatDouble(comparison.gaussian_normalized_residual_by_voxel) << ','
      << FormatDouble(comparison.shell_normalized_residual_by_voxel) << ','
      << FormatDouble(comparison.gaussian_normalized_residual_by_spread) << ','
      << FormatDouble(comparison.shell_normalized_residual_by_spread) << ','
      << FormatDouble(comparison.normalized_residual_gap) << ','
      << comparison.local_neighborhood_point_count << ','
      << (comparison.degenerate ? "true" : "false") << '\n';
  }
}

void WriteNormalizedComparisonCsv(
  const fs::path& output_dir,
  const std::vector<PrimitiveVoxelComparison>& comparisons)
{
  std::ofstream stream(output_dir / "voxel_comparison_normalized.csv");
  stream
    << "voxel_id,selection_tag,base_label,point_count,characteristic_spread_scale,"
    << "gaussian_average_center_distance,shell_geometric_average_residual,"
    << "gaussian_normalized_residual_by_voxel,shell_normalized_residual_by_voxel,"
    << "gaussian_normalized_residual_by_spread,shell_normalized_residual_by_spread,"
    << "normalized_residual_gap\n";

  for (const auto& comparison : comparisons)
  {
    stream
      << VoxelId(comparison.key) << ','
      << comparison.selection_tag << ','
      << comparison.base_label << ','
      << comparison.point_count << ','
      << FormatDouble(comparison.characteristic_spread_scale) << ','
      << FormatDouble(comparison.gaussian_average_center_distance) << ','
      << FormatDouble(comparison.shell_geometric_average_residual) << ','
      << FormatDouble(comparison.gaussian_normalized_residual_by_voxel) << ','
      << FormatDouble(comparison.shell_normalized_residual_by_voxel) << ','
      << FormatDouble(comparison.gaussian_normalized_residual_by_spread) << ','
      << FormatDouble(comparison.shell_normalized_residual_by_spread) << ','
      << FormatDouble(comparison.normalized_residual_gap) << '\n';
  }
}

void WriteRegistrationQuickcheckCsv(
  const fs::path& output_dir,
  const std::vector<PrimitiveRegistrationQuickcheck>& quickchecks)
{
  std::ofstream stream(output_dir / "registration_quickcheck.csv");
  stream
    << "voxel_id,key_x,key_y,key_z,selection_tag,base_label,perturbation,"
    << "neighborhood_point_count,reference_point_count,source_point_count,"
    << "perturbation_translation_norm,perturbation_rotation_degrees,"
    << "gaussian_nominal_score,gaussian_perturbed_score,gaussian_delta,"
    << "shell_nominal_score,shell_perturbed_score,shell_delta,delta_advantage,"
    << "shell_better_discrimination\n";

  for (const auto& quickcheck : quickchecks)
  {
    stream
      << quickcheck.voxel_id << ','
      << quickcheck.key.x << ','
      << quickcheck.key.y << ','
      << quickcheck.key.z << ','
      << quickcheck.selection_tag << ','
      << quickcheck.base_label << ','
      << quickcheck.perturbation << ','
      << quickcheck.neighborhood_point_count << ','
      << quickcheck.reference_point_count << ','
      << quickcheck.source_point_count << ','
      << FormatDouble(quickcheck.perturbation_translation_norm) << ','
      << FormatDouble(quickcheck.perturbation_rotation_degrees) << ','
      << FormatDouble(quickcheck.gaussian_nominal_score) << ','
      << FormatDouble(quickcheck.gaussian_perturbed_score) << ','
      << FormatDouble(quickcheck.gaussian_delta) << ','
      << FormatDouble(quickcheck.shell_nominal_score) << ','
      << FormatDouble(quickcheck.shell_perturbed_score) << ','
      << FormatDouble(quickcheck.shell_delta) << ','
      << FormatDouble(quickcheck.delta_advantage) << ','
      << (quickcheck.shell_better_discrimination ? "true" : "false") << '\n';
  }
}

void WriteSummary(
  const fs::path& output_dir,
  const PrimitiveValidationConfig& config,
  std::size_t input_points,
  const std::vector<std::string>& loaded_files,
  const PrimitiveValidationRun& run)
{
  std::ofstream stream(output_dir / "summary.md");
  stream << "# Direct Primitive Validation Stage-2 Quickcheck\n\n";
  stream << "- input_path: `" << config.input_path << "`\n";
  stream << "- output_dir: `" << config.output_dir << "`\n";
  stream << "- input_points: `" << input_points << "`\n";
  stream << "- loaded_files: `" << loaded_files.size() << "`\n";
  stream << "- voxel_size: `" << config.voxel_size << "`\n";
  stream << "- min_points_per_voxel: `" << config.min_points_per_voxel << "`\n";
  stream << "- max_voxels: `" << config.max_voxels << "`\n";
  stream << "- per_category_limit: `" << config.per_category_limit << "`\n";
  stream << "- selection_mode: `" << config.selection_mode << "`\n";
  stream << "- registration_neighborhood_voxels: `" << config.registration_neighborhood_voxels << "`\n";
  stream << "- registration_translation_step_ratio: `" << config.registration_translation_step_ratio << "`\n";
  stream << "- registration_rotation_degrees: `" << config.registration_rotation_degrees << "`\n";
  stream << "- gaussian_model: `mean/covariance baseline`\n";
  stream << "- surface_model: `PCA local frame + quantile axis scales + L_p shell scaffold`\n";
  stream << "- note: boundary/mixed category는 semantic relabel이 아니라 `linear` voxel proxy를 사용한다.\n\n";

  stream << "## Normalization Definition\n\n";
  stream << "- `characteristic_spread_scale = mean(sqrt(eigenvalues))`\n";
  stream << "- `shell_geometric_average_residual = shell_average_residual * mean(shell_axis_scales)`\n";
  stream << "- `gaussian_normalized_residual_by_spread = average_center_distance / characteristic_spread_scale`\n";
  stream << "- `shell_normalized_residual_by_spread = shell_geometric_average_residual / characteristic_spread_scale`\n";
  stream << "- shell 쪽 geometric residual은 exact point-to-surface distance가 아니라 `|r-1| * mean(axis_scales)` 기반 근사다.\n\n";

  struct NormalizedAggregate
  {
    int count = 0;
    double gaussian_by_voxel = 0.0;
    double shell_by_voxel = 0.0;
    double gaussian_by_spread = 0.0;
    double shell_by_spread = 0.0;
    double gap = 0.0;
  };

  std::unordered_map<std::string, NormalizedAggregate> normalized_aggregates;
  for (const auto& comparison : run.comparisons)
  {
    auto& aggregate = normalized_aggregates[comparison.selection_tag];
    ++aggregate.count;
    aggregate.gaussian_by_voxel += comparison.gaussian_normalized_residual_by_voxel;
    aggregate.shell_by_voxel += comparison.shell_normalized_residual_by_voxel;
    aggregate.gaussian_by_spread += comparison.gaussian_normalized_residual_by_spread;
    aggregate.shell_by_spread += comparison.shell_normalized_residual_by_spread;
    aggregate.gap += comparison.normalized_residual_gap;
  }

  stream << "## Normalized Comparison Means\n\n";
  stream << "| selection_tag | count | gaussian_by_voxel | shell_by_voxel | gaussian_by_spread | shell_by_spread | gaussian_minus_shell |\n";
  stream << "| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n";
  std::vector<std::string> ordered_tags;
  ordered_tags.reserve(normalized_aggregates.size());
  for (const auto& item : normalized_aggregates)
  {
    ordered_tags.push_back(item.first);
  }
  std::sort(ordered_tags.begin(), ordered_tags.end());
  for (const auto& tag : ordered_tags)
  {
    const auto& aggregate = normalized_aggregates.at(tag);
    const double denom = static_cast<double>(std::max(aggregate.count, 1));
    stream
      << "| " << tag
      << " | " << aggregate.count
      << " | " << FormatDouble(aggregate.gaussian_by_voxel / denom)
      << " | " << FormatDouble(aggregate.shell_by_voxel / denom)
      << " | " << FormatDouble(aggregate.gaussian_by_spread / denom)
      << " | " << FormatDouble(aggregate.shell_by_spread / denom)
      << " | " << FormatDouble(aggregate.gap / denom)
      << " |\n";
  }

  struct QuickAggregate
  {
    int count = 0;
    double gaussian_delta = 0.0;
    double shell_delta = 0.0;
    double advantage = 0.0;
    int shell_better = 0;
  };

  std::unordered_map<std::string, QuickAggregate> quick_aggregates;
  for (const auto& quickcheck : run.registration_quickchecks)
  {
    const std::string key = quickcheck.selection_tag + "|" + quickcheck.perturbation;
    auto& aggregate = quick_aggregates[key];
    ++aggregate.count;
    aggregate.gaussian_delta += quickcheck.gaussian_delta;
    aggregate.shell_delta += quickcheck.shell_delta;
    aggregate.advantage += quickcheck.delta_advantage;
    if (quickcheck.shell_better_discrimination)
    {
      ++aggregate.shell_better;
    }
  }

  stream << "\n## Registration Quickcheck Means\n\n";
  stream << "| selection_tag | perturbation | cases | gaussian_delta | shell_delta | shell_minus_gaussian | shell_better_cases |\n";
  stream << "| --- | --- | ---: | ---: | ---: | ---: | ---: |\n";
  std::vector<std::string> ordered_quick_keys;
  ordered_quick_keys.reserve(quick_aggregates.size());
  for (const auto& item : quick_aggregates)
  {
    ordered_quick_keys.push_back(item.first);
  }
  std::sort(ordered_quick_keys.begin(), ordered_quick_keys.end());
  for (const auto& key : ordered_quick_keys)
  {
    const auto separator = key.find('|');
    const std::string selection_tag = key.substr(0, separator);
    const std::string perturbation = key.substr(separator + 1);
    const auto& aggregate = quick_aggregates.at(key);
    const double denom = static_cast<double>(std::max(aggregate.count, 1));
    stream
      << "| " << selection_tag
      << " | " << perturbation
      << " | " << aggregate.count
      << " | " << FormatDouble(aggregate.gaussian_delta / denom)
      << " | " << FormatDouble(aggregate.shell_delta / denom)
      << " | " << FormatDouble(aggregate.advantage / denom)
      << " | " << aggregate.shell_better
      << " |\n";
  }

  stream << "\n## Selected Cases\n\n";
  for (const auto& comparison : run.comparisons)
  {
    stream
      << "- `" << VoxelId(comparison.key) << "`"
      << " | " << comparison.selection_tag
      << " | base_label=`" << comparison.base_label << "`"
      << " | points=`" << comparison.point_count << "`"
      << " | neighborhood_points=`" << comparison.local_neighborhood_point_count << "`"
      << " | gaussian_by_spread=`" << FormatDouble(comparison.gaussian_normalized_residual_by_spread) << "`"
      << " | shell_by_spread=`" << FormatDouble(comparison.shell_normalized_residual_by_spread) << "`"
      << " | gap=`" << FormatDouble(comparison.normalized_residual_gap) << "`\n";
  }

  stream << "\n## Interpretation Guardrails\n\n";
  stream << "- 이 quickcheck는 exact GES/GND fitting이 아니라 surface-shell proxy와 Gaussian baseline의 초기 분별력 검사용이다.\n";
  stream << "- registration quickcheck는 local neighborhood를 reference/source로 deterministic split한 뒤 nominal/perturbed score 변화를 보는 소규모 offline benchmark다.\n";
  stream << "- quickcheck score는 model-internal normalized residual이므로, full registration accuracy나 SLAM 성능을 아직 주장하지 않는다.\n";
  stream << "- shell 모델이 promising하다고 보려면 normalized residual에서 category별 이득이 반복되고, perturbation delta에서도 shell 쪽 분별력이 일관되게 커야 한다.\n";
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

PrimitiveValidationRun RunDirectPrimitiveValidation(
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

  std::vector<PrimitiveVoxelBucket> buckets;
  buckets.reserve(voxel_map.size());
  for (const auto& item : voxel_map)
  {
    buckets.push_back(item.second);
  }
  std::sort(
    buckets.begin(),
    buckets.end(),
    [](const PrimitiveVoxelBucket& lhs, const PrimitiveVoxelBucket& rhs)
    {
      return VoxelKeyLess(lhs.key, rhs.key);
    });

  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> lookup;
  lookup.reserve(buckets.size());
  for (std::size_t index = 0; index < buckets.size(); ++index)
  {
    lookup[buckets[index].key] = index;
  }

  std::vector<PrimitiveModelBundle> all_models;
  all_models.reserve(buckets.size());
  std::vector<PrimitiveVoxelComparison> all_comparisons;
  all_comparisons.reserve(buckets.size());

  for (const auto& bucket : buckets)
  {
    if (static_cast<int>(bucket.points.size()) < config.min_points_per_voxel)
    {
      continue;
    }

    PrimitiveModelBundle model;
    model.bucket = bucket;
    model.stats = ComputePrimitiveStats(bucket);
    model.gaussian = FitGaussianPrimitive(bucket, model.stats, config.gaussian_regularization);
    model.shell = FitSurfaceShellPrimitive(
      bucket,
      model.stats,
      config.shell_axis_scale_quantile,
      config.shell_axis_scale_min,
      config.shell_shape_exponent);

    PrimitiveVoxelComparison comparison;
    comparison.key = bucket.key;
    comparison.voxel_center = bucket.center;
    comparison.mean = model.stats.mean;
    comparison.eigenvalues = model.stats.eigenvalues;
    comparison.point_count = model.stats.num_points;
    comparison.base_label = model.stats.label;
    comparison.linearity = model.stats.linearity;
    comparison.planarity = model.stats.planarity;
    comparison.scattering = model.stats.scattering;
    comparison.anisotropy = model.stats.anisotropy;
    comparison.omnivariance = model.stats.omnivariance;
    comparison.gaussian_average_mahalanobis2 = model.gaussian.average_mahalanobis2;
    comparison.gaussian_voxel_center_mahalanobis2 = model.gaussian.voxel_center_mahalanobis2;
    comparison.gaussian_average_center_distance = model.gaussian.average_center_distance;
    comparison.gaussian_condition_ratio = model.gaussian.condition_ratio;
    comparison.surface_normal_rms = model.shell.normal_rms;
    comparison.shell_average_residual = model.shell.average_shell_residual;
    comparison.shell_voxel_center_residual = model.shell.voxel_center_shell_residual;
    comparison.shell_average_radius = model.shell.average_radius;
    comparison.shell_axis_scales = model.shell.axis_scales;
    comparison.shell_axis_condition = model.shell.axis_condition;
    comparison.sparsity_indicator =
      1.0 / std::sqrt(static_cast<double>(std::max<std::size_t>(comparison.point_count, 1)));
    comparison.degenerate = model.stats.degenerate;
    FillNormalizedMetrics(&comparison, model.stats, model.shell, config.voxel_size);

    model.comparison = comparison;
    all_models.push_back(model);
    all_comparisons.push_back(comparison);
  }

  const std::vector<std::size_t> selected_indices = SelectCandidateIndices(all_comparisons, config);
  PrimitiveValidationRun run;
  run.comparisons.reserve(selected_indices.size());

  for (const std::size_t index : selected_indices)
  {
    PrimitiveVoxelComparison comparison = all_models[index].comparison;
    comparison.selection_tag = SelectionTagFromBaseLabel(comparison.base_label);
    comparison.selection_note = SelectionNote(comparison);
    AppendRegistrationQuickchecks(
      &run.registration_quickchecks,
      &comparison,
      lookup,
      buckets,
      config);
    run.comparisons.push_back(comparison);
  }

  std::sort(
    run.comparisons.begin(),
    run.comparisons.end(),
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

  std::sort(
    run.registration_quickchecks.begin(),
    run.registration_quickchecks.end(),
    [](const PrimitiveRegistrationQuickcheck& lhs, const PrimitiveRegistrationQuickcheck& rhs)
    {
      if (lhs.selection_tag != rhs.selection_tag)
      {
        return lhs.selection_tag < rhs.selection_tag;
      }
      if (lhs.voxel_id != rhs.voxel_id)
      {
        return lhs.voxel_id < rhs.voxel_id;
      }
      return lhs.perturbation < rhs.perturbation;
    });

  return run;
}

void SaveDirectPrimitiveValidationResults(
  const PrimitiveValidationRun& run,
  const PrimitiveValidationConfig& config,
  std::size_t input_points,
  const std::vector<std::string>& loaded_files)
{
  const fs::path output_dir(config.output_dir);
  fs::create_directories(output_dir);

  WriteLoadedFiles(output_dir, loaded_files);
  WriteSelectedCasesCsv(output_dir, run.comparisons);
  WriteComparisonCsv(output_dir, run.comparisons);
  WriteNormalizedComparisonCsv(output_dir, run.comparisons);
  WriteRegistrationQuickcheckCsv(output_dir, run.registration_quickchecks);
  WriteSummary(output_dir, config, input_points, loaded_files, run);
}

}  // namespace ges_voxel_mapping
