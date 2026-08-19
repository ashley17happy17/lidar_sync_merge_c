#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <lidar_utils/cloud_utils.hpp>
#include "lidar_dynamicmerge_c/types.hpp"
#include "lidar_dynamicmerge_c/logger.hpp"
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
  void loadImuData();
  void loadOdomData();
  void loadLidarContent();
  void processFrame(const GNSSData &gnss, const GNSSData &next_gnss,
                    const std::map<int, std::string> &matched_files);
  // Vehicle pose in the local world frame at an arbitrary epoch, obtained by
  // interpolating the GNSS trajectory (lerp on position, slerp on attitude).
  // Clamps to the first/last record outside the trajectory.
  Eigen::Matrix4d interpolatePoseAt(double t) const;

  YAML::Node config_;
  std::vector<GNSSData> gnss_data_list_;
  // Lightweight position-only view of the GNSS trajectory, used as the
  // translation source for motion compensation.
  std::vector<lidar_utils::GnssSample> gnss_samples_;
  // IMU gyro (+ optional accel) samples in the GNSS/vehicle FLU frame: the
  // rotation source always, and the translation source for IMU_ACC.
  std::vector<lidar_utils::ImuSample> imu_data_list_;
  // Odometer velocity samples (vehicle FLU frame): translation source for ODOM.
  std::vector<lidar_utils::OdomSample> odom_data_list_;

  // Config parameters
  std::string gnss_file_;
  std::string imu_file_;
  std::string odom_file_;
  // Factor applied to the odom_file velocity columns to reach m/s (1.0 for a
  // file already in m/s, 1/3.6 for a CAN speed logged in km/h).
  double odom_scale_ = 1.0;
  // Translation source for motion compensation (rotation is always the gyro).
  lidar_utils::MotionMethod motion_method_ =
      lidar_utils::MotionMethod::GNSS_TRANS;
  // Use the legacy deskew instead: per-point GNSS pose interpolated between the
  // current and next fix, no IMU. Kept for comparison against the newer methods;
  // motion_method_ is unused when this is set.
  bool use_legacy_motion_ = false;
  double gnss_std_thres_;
  double gnss_freq_;
  double lidar_hz_;

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