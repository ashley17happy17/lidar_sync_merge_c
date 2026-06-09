#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <lidar_utils/cloud_utils.hpp>
#include <lidar_utils/types.hpp>
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

struct LidarConfig {
  std::string path;
  lidar_utils::FileFormat format;
  std::string type;
  Eigen::Vector3d trans;
  Eigen::Vector3d rot; // roll, pitch, yaw or similar. According to params, it's
                       // [roll, pitch, yaw] in degree? Wait, bs is 3 values,
                       // usually roll pitch yaw or yaw pitch roll.
};

struct GNSSData {
  double timestamp;
  Eigen::VectorXd pose_6d; // [x, y, z, roll, pitch, yaw]
  Eigen::Vector3d position_std;
  double geoidSep;

  // Helper to get 4x4 transformation matrix
  Eigen::Matrix4d getTransform() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (pose_6d.size() >= 6) {
      double r = pose_6d[3] * M_PI / 180.0;
      double p = pose_6d[4] * M_PI / 180.0;
      double y = pose_6d[5] * M_PI / 180.0;
      Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
      T.block<3, 3>(0, 0) = q.matrix();
      T.block<3, 1>(0, 3) = pose_6d.segment<3>(0);
    }
    return T;
  }
};

class LiDARDynamicMerge {
public:
  LiDARDynamicMerge(const std::string &config_file);
  ~LiDARDynamicMerge() = default;

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