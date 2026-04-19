#include "ges_voxel_mapping/direct_primitive_validation.hpp"

#include <cctype>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

std::string NormalizeSelectionMode(std::string mode)
{
  for (char& ch : mode)
  {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  if (mode.empty())
  {
    return "auto_small";
  }
  if (mode != "auto_small")
  {
    throw std::runtime_error("Unsupported selection mode: " + mode + ". Use auto_small.");
  }
  return mode;
}

void PrintUsage()
{
  std::cout
    << "Usage: voxel_primitive_comparison --input <pcd|dir> [--output <dir>]\n"
    << "       [--voxel-size <float>] [--min-points-per-voxel <int>] [--max-points <int>]\n"
    << "       [--max-voxels <int>] [--per-category-limit <int>] [--selection-mode auto_small]\n"
    << "       [--shell-shape-exponent <float>] [--shell-axis-scale-quantile <float>]\n"
    << "       [--registration-neighborhood-voxels <int>] [--registration-min-points <int>]\n"
    << "       [--registration-translation-step-ratio <float>] [--registration-rotation-degrees <float>]\n";
}

}  // namespace

int main(int argc, char** argv)
{
  try
  {
    ges_voxel_mapping::PrimitiveValidationConfig config;

    for (int index = 1; index < argc; ++index)
    {
      const std::string arg(argv[index]);
      if (arg == "--help" || arg == "-h")
      {
        PrintUsage();
        return 0;
      }
      if ((arg == "--input" || arg == "-i") && index + 1 < argc)
      {
        config.input_path = argv[++index];
        continue;
      }
      if ((arg == "--output" || arg == "-o") && index + 1 < argc)
      {
        config.output_dir = argv[++index];
        continue;
      }
      if (arg == "--voxel-size" && index + 1 < argc)
      {
        config.voxel_size = std::stod(argv[++index]);
        continue;
      }
      if (arg == "--min-points-per-voxel" && index + 1 < argc)
      {
        config.min_points_per_voxel = std::stoi(argv[++index]);
        continue;
      }
      if (arg == "--max-points" && index + 1 < argc)
      {
        config.max_points = std::stoi(argv[++index]);
        continue;
      }
      if (arg == "--max-voxels" && index + 1 < argc)
      {
        config.max_voxels = std::stoi(argv[++index]);
        continue;
      }
      if (arg == "--per-category-limit" && index + 1 < argc)
      {
        config.per_category_limit = std::stoi(argv[++index]);
        continue;
      }
      if (arg == "--selection-mode" && index + 1 < argc)
      {
        config.selection_mode = NormalizeSelectionMode(argv[++index]);
        continue;
      }
      if (arg == "--gaussian-regularization" && index + 1 < argc)
      {
        config.gaussian_regularization = std::stod(argv[++index]);
        continue;
      }
      if (arg == "--shell-axis-scale-quantile" && index + 1 < argc)
      {
        config.shell_axis_scale_quantile = std::stod(argv[++index]);
        continue;
      }
      if (arg == "--shell-axis-scale-min" && index + 1 < argc)
      {
        config.shell_axis_scale_min = std::stod(argv[++index]);
        continue;
      }
      if (arg == "--shell-shape-exponent" && index + 1 < argc)
      {
        config.shell_shape_exponent = std::stod(argv[++index]);
        continue;
      }
      if (arg == "--registration-neighborhood-voxels" && index + 1 < argc)
      {
        config.registration_neighborhood_voxels = std::stoi(argv[++index]);
        continue;
      }
      if (arg == "--registration-min-points" && index + 1 < argc)
      {
        config.registration_min_points = std::stoi(argv[++index]);
        continue;
      }
      if (arg == "--registration-translation-step-ratio" && index + 1 < argc)
      {
        config.registration_translation_step_ratio = std::stod(argv[++index]);
        continue;
      }
      if (arg == "--registration-rotation-degrees" && index + 1 < argc)
      {
        config.registration_rotation_degrees = std::stod(argv[++index]);
        continue;
      }

      throw std::runtime_error("Unknown or incomplete argument: " + arg);
    }

    config.selection_mode = NormalizeSelectionMode(config.selection_mode);

    if (config.input_path.empty())
    {
      throw std::runtime_error("Pass --input <pcd|dir>.");
    }
    if (config.voxel_size <= 0.0)
    {
      throw std::runtime_error("voxel_size must be positive.");
    }
    if (config.min_points_per_voxel < 3)
    {
      throw std::runtime_error("min_points_per_voxel must be >= 3.");
    }
    if (config.registration_neighborhood_voxels < 0)
    {
      throw std::runtime_error("registration_neighborhood_voxels must be >= 0.");
    }
    if (config.registration_min_points < 6)
    {
      throw std::runtime_error("registration_min_points must be >= 6.");
    }

    std::vector<std::string> loaded_files;
    const ges_voxel_mapping::PointCloud::Ptr cloud =
      ges_voxel_mapping::LoadPrimitiveValidationCloud(config, &loaded_files);
    const ges_voxel_mapping::PrimitiveValidationRun run =
      ges_voxel_mapping::RunDirectPrimitiveValidation(*cloud, config);
    ges_voxel_mapping::SaveDirectPrimitiveValidationResults(
      run,
      config,
      cloud->size(),
      loaded_files);

    std::cout << "[done] input=" << config.input_path
              << " points=" << cloud->size()
              << " selected_voxels=" << run.comparisons.size()
              << " quickchecks=" << run.registration_quickchecks.size()
              << " output=" << config.output_dir << '\n';
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << "voxel_primitive_comparison failed: " << error.what() << '\n';
    return 1;
  }
}
