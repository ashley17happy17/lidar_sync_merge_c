#include "lidar_dynamicmerge_c/dynamic_merge_node.hpp"
#include <iostream>

namespace lidar_dynamic_merge {

void DynamicMergeNode::loadConfig(const std::string &config_file) {
  config_ = YAML::LoadFile(config_file);

  gnss_file_ = config_["gnss_file"].as<std::string>();
  gnss_std_thres_ = config_["gnss_std_thres"].as<double>();
  gnss_freq_ = config_["gnss_freq"].as<double>();

  // Load lidar configs (1 to 5)
  for (int i = 1; i <= 5; ++i) {
    std::string prefix = "lidar" + std::to_string(i) + "_";
    if (config_[prefix + "fp"]) {
      std::string path = config_[prefix + "fp"].as<std::string>();
      if (path.empty()) {
        LOG_WARN("Skipping lidar " << i << " because fp is empty.");
        continue;
      }

      LidarConfig lc;
      lc.path = path;
      if (config_[prefix + "type"]) {
        std::string type_str = config_[prefix + "type"].as<std::string>();
        if (type_str == "OUSTER_OS1_128")
          lc.type = lidar_utils::SensorType::OUSTER_OS1_128;
        else if (type_str == "OUSTER_OS1_32")
          lc.type = lidar_utils::SensorType::OUSTER_OS1_32;
        else if (type_str == "VELODYNE_VLP16")
          lc.type = lidar_utils::SensorType::VELODYNE_VLP16;
        else if (type_str == "VELODYNE_VLS128")
          lc.type = lidar_utils::SensorType::VELODYNE_VLS128;
        else {
          LOG_ERROR("Config: Wrong LiDAR Type input.");
          exit(1);
        }
      } else {
        LOG_ERROR("Config: Wrong LiDAR Type input.");
        exit(1);
      }
      if (config_[prefix + "format"]) {
        std::string fmt_str = config_[prefix + "format"].as<std::string>();
        if (fmt_str == "PCD_ASCII")
          lc.format = lidar_utils::FileFormat::PCD_ASCII;
        else if (fmt_str == "LAS")
          lc.format = lidar_utils::FileFormat::LAS;
        else
          lc.format = lidar_utils::FileFormat::PCD_BINARY;
      } else {
        lc.format = lidar_utils::FileFormat::PCD_BINARY;
      }

      auto la = config_[prefix + "la"].as<std::vector<double>>();
      auto bs = config_[prefix + "bs"].as<std::vector<double>>();
      lc.trans = Eigen::Vector3d(la[0], la[1], la[2]);
      lc.rot = Eigen::Vector3d(bs[0], bs[1], bs[2]);

      lidars_[i] = lc;
    }
  }

  if (lidars_.empty()) {
    LOG_ERROR("All lidar file paths are empty. Exiting.");
    exit(1);
  }

  crop_min_ = config_["crop_min_bound"].as<std::vector<double>>();
  crop_max_ = config_["crop_max_bound"].as<std::vector<double>>();
  denoise_radius_ = config_["denoise_radius"].as<double>();
  denoise_epsilon_ = config_["denoise_epsilon"].as<double>();
  ds_voxel_size_ = config_["ds_voxel_size"].as<double>();
  motion_enable_ = config_["motion_enable"].as<bool>();
  merge_voxel_size_ = config_["merge_voxel_size"].as<double>();

  out_lidar_ = config_["out_lidar"].as<int>();
  out_fp_ = config_["out_fp"].as<std::string>();
  std::string fmt = config_["out_format"].as<std::string>();
  if (fmt == "PCD_ASCII")
    out_format_ = lidar_utils::FileFormat::PCD_ASCII;
  else if (fmt == "LAS")
    out_format_ = lidar_utils::FileFormat::LAS;
  else
    out_format_ = lidar_utils::FileFormat::PCD_BINARY;

  auto out_la = config_["out_la"].as<std::vector<double>>();
  auto out_bs = config_["out_bs"].as<std::vector<double>>();
  out_la_ = Eigen::Vector3d(out_la[0], out_la[1], out_la[2]);
  out_bs_ = Eigen::Vector3d(out_bs[0], out_bs[1], out_bs[2]);
}

} // namespace lidar_dynamic_merge
