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
    << "  auto      : file면 single, dir면 aggregate\n"
    << "  single    : 단일 PCD 하나 분석\n"
    << "  aggregate : 디렉토리의 PCD를 모두 누적해서 하나의 map처럼 분석\n"
    << "  batch     : 디렉토리의 각 PCD를 개별 분석\n";
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
    bool output_overridden = false;

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

    config.input_mode = NormalizeMode(config.input_mode);
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
