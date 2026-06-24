#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <lidar_utils/cloud_utils.hpp>
#include "lidar_dynamicmerge_c/types.hpp"
#include "lidar_dynamicmerge_c/time_sync.hpp"
#include "lidar_dynamicmerge_c/gnss_processor.hpp"
#include <map>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace lidar_dynamic_merge {



class DynamicMergeNode {
public:
  DynamicMergeNode(const std::string &config_file);
  ~DynamicMergeNode() = default;

  void run();

private:
  void loadConfig(const std::string &config_file);
  void loadGnssData();
  void loadLidarContent();
  void processFrame(const GNSSData &gnss, const GNSSData *next_gnss,
                    const std::map<int, std::string> &matched_files);

  YAML::Node config_;
  std::vector<GNSSData> gnss_data_list_;

  // Config parameters
  std::string gnss_file_;
  double gnss_std_thres_;
  double gnss_freq_;

  std::map<int, LidarConfig> lidars_;
  std::map<int, std::vector<lidar_utils::LidarContent>>
      lidar_files_; // Lidar ID -> files sorted by timestamp

  std::vector<double> crop_min_;
  std::vector<double> crop_max_;
  double denoise_radius_;
  double denoise_epsilon_;
  double ds_voxel_size_;
  bool motion_enable_;
  double merge_voxel_size_;
  int out_lidar_;
  std::string out_fp_;
  lidar_utils::FileFormat out_format_;
  Eigen::Vector3d out_la_;
  Eigen::Vector3d out_bs_;
};

} // namespace lidar_dynamic_merge