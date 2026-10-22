#pragma once

#include <Eigen/Dense>
#include <lidar_utils/cloud_utils.hpp>
#include <lidar_utils/types.hpp>
#include <map>
#include <string>
#include <vector>
#include <cmath>

namespace lidar_dynamic_merge {

struct LidarConfig {
  std::string source;
  lidar_utils::FileFormat format;
  lidar_utils::SensorType type;
  Eigen::Vector3d trans;
  Eigen::Vector3d rot; // roll, pitch, yaw or similar.
  // Seconds ADDED to this LiDAR's timestamps (both the filename time used for
  // GNSS synchronisation and the per-point time used for deskew) to put them on
  // the GNSS clock. A LiDAR clock running 1.145 s fast needs -1.145.
  double time_offset = 0.0;
};

struct ImuOrientation {
  double timestamp = 0.0;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

struct GNSSData {
  double timestamp = 0.0;
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  double twd97_x = 0.0;
  double twd97_y = 0.0;
  double twd97_z = 0.0;
  double raw_roll = 0.0;  // degrees (from GNSS/INS record)
  double raw_pitch = 0.0; // degrees (from GNSS/INS record)
  double raw_yaw = 0.0;   // degrees (from GNSS/INS record)
  Eigen::VectorXd pose_6d; // [x, y, z, roll, pitch, yaw]
  Eigen::Vector3d position_std = Eigen::Vector3d::Zero();
  double geoidSep = 0.0;

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

struct SyncFrame {
  GNSSData curr_gnss;
  GNSSData next_gnss;
  std::map<int, std::string> matched_files;
  std::string time_stem;
  double lidar_time = 0.0; // exact LiDAR timestamp in seconds
};

} // namespace lidar_dynamic_merge
