#include "ges_voxel_mapping/voxel_analysis.hpp"

#include <cctype>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

#ifdef GES_DEFAULT_CONFIG_PATH
constexpr const char* kDefaultConfigPath = GES_DEFAULT_CONFIG_PATH;
#else
constexpr const char* kDefaultConfigPath = "src/ges_voxel_mapping/config/voxel_morphology.yaml";
#endif

std::string NormalizeMode(std::string mode)
{
  for (char& ch : mode)
  {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  if (mode.empty())
  {
    return "auto";
  }
  if (mode != "auto" && mode != "single" && mode != "aggregate" && mode != "batch")
  {
    throw std::runtime_error("Unsupported mode: " + mode + ". Use auto|single|aggregate|batch.");
  }
  return mode;
}

std::string NormalizeRankingMode(std::string mode)
{
  for (char& ch : mode)
  {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
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
      "Unsupported ranking mode: " + mode +
      ". Use score_only|corner_priority|nonplanar_priority|context_face_support|"
      "context_normal_variation|context_asymmetry|context_hybrid.");
  }
  return mode;
}

bool ParseBool(const std::string& text)
{
  std::string normalized = text;
  for (char& ch : normalized)
  {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on")
  {
    return true;
  }
  if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off")
  {
    return false;
  }
  throw std::runtime_error("Failed to parse boolean value: " + text);
}

std::string PackageConfigPath()
{
  return kDefaultConfigPath;
}

std::string StemFromPath(const std::string& path)
{
  return fs::path(path).stem().string();
}

std::string ResolveSingleOutputDir(
  const std::string& base_output_dir,
  const std::string& input_path,
  bool output_overridden)
{
  if (output_overridden)
  {
    return base_output_dir;
  }
  return (fs::path(base_output_dir) / StemFromPath(input_path)).string();
}

void PrintUsage()
{
  std::cout
    << "Usage: voxel_morphology_analyzer [--config <yaml>] --input <pcd|dir> [--output <dir>] [--mode auto|single|aggregate|batch]\n"
    << "       [--voxel-size <float>] [--min-points-per-voxel <int>] [--shape-exponent <float>]\n"
    << "       [--axis-scale-quantile <float>] [--ranking-mode <mode>] [--save-top-k <int>]\n"
    << "       [--export-interesting-voxels <bool>] [--export-top-k-pcd <int>]\n"
    << "  auto      : file면 single, dir면 aggregate\n"
    << "  single    : 단일 PCD 하나 분석\n"
    << "  aggregate : 디렉토리의 PCD를 모두 누적해서 하나의 map처럼 분석\n"
    << "  batch     : 디렉토리의 각 PCD를 개별 분석\n"
    << "  ranking   : score_only|corner_priority|nonplanar_priority|context_face_support|"
       "context_normal_variation|context_asymmetry|context_hybrid\n";
}

void AnalyzeAndSave(
  const std::string& input_path,
  const std::string& output_dir,
  const ges_voxel_mapping::AnalysisConfig& base_config)
{
  ges_voxel_mapping::AnalysisConfig run_config = base_config;
  run_config.input_path = input_path;
  run_config.output_dir = output_dir;

  std::vector<std::string> loaded_files;
  const ges_voxel_mapping::PointCloud::Ptr cloud =
    ges_voxel_mapping::LoadInputCloud(run_config, &loaded_files);
  const std::vector<ges_voxel_mapping::VoxelMetrics> metrics =
    ges_voxel_mapping::RunVoxelMorphologyAnalysis(*cloud, run_config);

  ges_voxel_mapping::SaveAnalysisResults(metrics, run_config, cloud->size(), loaded_files, cloud.get());

  std::cout << "[done] input=" << input_path
            << " points=" << cloud->size()
            << " voxels=" << metrics.size()
            << " output=" << output_dir << '\n';
}

}  // namespace

int main(int argc, char** argv)
{
  try
  {
    std::string config_path = PackageConfigPath();
    std::string input_override;
    std::string output_override;
    std::string mode_override;
    std::string ranking_mode_override;
    bool output_overridden = false;
    bool voxel_size_overridden = false;
    bool min_points_overridden = false;
    bool shape_exponent_overridden = false;
    bool axis_scale_quantile_overridden = false;
    bool save_top_k_overridden = false;
    bool export_interesting_voxels_overridden = false;
    bool export_top_k_pcd_overridden = false;
    double voxel_size_override = 0.0;
    int min_points_override = 0;
    double shape_exponent_override = 0.0;
    double axis_scale_quantile_override = 0.0;
    int save_top_k_override = 0;
    bool export_interesting_voxels_override = true;
    int export_top_k_pcd_override = 0;

    for (int index = 1; index < argc; ++index)
    {
      const std::string arg(argv[index]);
      if (arg == "--help" || arg == "-h")
      {
        PrintUsage();
        return 0;
      }
      if ((arg == "--config" || arg == "-c") && index + 1 < argc)
      {
        config_path = argv[++index];
        continue;
      }
      if ((arg == "--input" || arg == "-i") && index + 1 < argc)
      {
        input_override = argv[++index];
        continue;
      }
      if ((arg == "--output" || arg == "-o") && index + 1 < argc)
      {
        output_override = argv[++index];
        output_overridden = true;
        continue;
      }
      if ((arg == "--mode" || arg == "-m") && index + 1 < argc)
      {
        mode_override = argv[++index];
        continue;
      }
      if (arg == "--voxel-size" && index + 1 < argc)
      {
        voxel_size_override = std::stod(argv[++index]);
        voxel_size_overridden = true;
        continue;
      }
      if (arg == "--min-points-per-voxel" && index + 1 < argc)
      {
        min_points_override = std::stoi(argv[++index]);
        min_points_overridden = true;
        continue;
      }
      if (arg == "--shape-exponent" && index + 1 < argc)
      {
        shape_exponent_override = std::stod(argv[++index]);
        shape_exponent_overridden = true;
        continue;
      }
      if (arg == "--axis-scale-quantile" && index + 1 < argc)
      {
        axis_scale_quantile_override = std::stod(argv[++index]);
        axis_scale_quantile_overridden = true;
        continue;
      }
      if (arg == "--ranking-mode" && index + 1 < argc)
      {
        ranking_mode_override = argv[++index];
        continue;
      }
      if (arg == "--save-top-k" && index + 1 < argc)
      {
        save_top_k_override = std::stoi(argv[++index]);
        save_top_k_overridden = true;
        continue;
      }
      if (arg == "--export-interesting-voxels" && index + 1 < argc)
      {
        export_interesting_voxels_override = ParseBool(argv[++index]);
        export_interesting_voxels_overridden = true;
        continue;
      }
      if (arg == "--export-top-k-pcd" && index + 1 < argc)
      {
        export_top_k_pcd_override = std::stoi(argv[++index]);
        export_top_k_pcd_overridden = true;
        continue;
      }

      throw std::runtime_error("Unknown or incomplete argument: " + arg);
    }

    ges_voxel_mapping::AnalysisConfig config = ges_voxel_mapping::LoadAnalysisConfig(config_path);
    if (!input_override.empty())
    {
      config.input_path = input_override;
    }
    if (!output_override.empty())
    {
      config.output_dir = output_override;
    }
    if (!mode_override.empty())
    {
      config.input_mode = mode_override;
    }
    if (voxel_size_overridden)
    {
      config.voxel_size = voxel_size_override;
    }
    if (min_points_overridden)
    {
      config.min_points_per_voxel = min_points_override;
    }
    if (shape_exponent_overridden)
    {
      config.shape_exponent = shape_exponent_override;
    }
    if (axis_scale_quantile_overridden)
    {
      config.axis_scale_quantile = axis_scale_quantile_override;
    }
    if (!ranking_mode_override.empty())
    {
      config.ranking_mode = NormalizeRankingMode(ranking_mode_override);
    }
    if (save_top_k_overridden)
    {
      config.save_top_k = save_top_k_override;
    }
    if (export_interesting_voxels_overridden)
    {
      config.export_interesting_voxels = export_interesting_voxels_override;
    }
    if (export_top_k_pcd_overridden)
    {
      config.export_top_k_pcd = export_top_k_pcd_override;
    }

    config.input_mode = NormalizeMode(config.input_mode);
    config.ranking_mode = NormalizeRankingMode(config.ranking_mode);
    if (config.voxel_size <= 0.0)
    {
      throw std::runtime_error("voxel_size must be positive");
    }
    if (config.min_points_per_voxel < 3)
    {
      throw std::runtime_error("min_points_per_voxel must be >= 3");
    }
    if (config.axis_scale_quantile < 0.0 || config.axis_scale_quantile > 1.0)
    {
      throw std::runtime_error("axis_scale_quantile must be in [0, 1]");
    }
    if (config.input_path.empty())
    {
      throw std::runtime_error("input_path is empty. Pass --input <pcd|dir>.");
    }

    const fs::path input_path(config.input_path);
    if (!fs::exists(input_path))
    {
      throw std::runtime_error("Input path does not exist: " + config.input_path);
    }

    std::string effective_mode = config.input_mode;
    if (effective_mode == "auto")
    {
      effective_mode = fs::is_directory(input_path) ? "aggregate" : "single";
    }

    if (effective_mode == "single")
    {
      if (fs::is_directory(input_path))
      {
        throw std::runtime_error("single mode expects a single PCD file, not a directory.");
      }
      AnalyzeAndSave(
        config.input_path,
        ResolveSingleOutputDir(config.output_dir, config.input_path, output_overridden),
        config);
      return 0;
    }

    if (effective_mode == "aggregate")
    {
      AnalyzeAndSave(config.input_path, config.output_dir, config);
      return 0;
    }

    if (effective_mode == "batch")
    {
      if (!fs::is_directory(input_path))
      {
        AnalyzeAndSave(
          config.input_path,
          ResolveSingleOutputDir(config.output_dir, config.input_path, output_overridden),
          config);
        return 0;
      }

      const std::vector<std::string> files = ges_voxel_mapping::CollectInputFiles(config.input_path);
      for (const auto& file : files)
      {
        const std::string output_dir = (fs::path(config.output_dir) / StemFromPath(file)).string();
        AnalyzeAndSave(file, output_dir, config);
      }
      std::cout << "[batch-done] count=" << files.size()
                << " input_dir=" << config.input_path
                << " output_root=" << config.output_dir << '\n';
      return 0;
    }

    throw std::runtime_error("Reached unexpected mode branch");
  }
  catch (const std::exception& error)
  {
    std::cerr << "voxel_morphology_analyzer failed: " << error.what() << '\n';
    return 1;
  }
}
