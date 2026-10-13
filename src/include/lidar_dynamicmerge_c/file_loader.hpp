#pragma once

#include <string>
#include <vector>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <lidar_utils/types.hpp>
#include "lidar_dynamicmerge_c/types.hpp"

namespace lidar_dynamic_merge {

class FileLoader {
public:
  // Loads GNSS CSV using gnss_processor functions
  static bool loadGnssCSV(const std::string &filepath,
                          std::vector<GNSSData> &gnss_list,
                          Eigen::Vector3d &local_origin);

  // Loads whitespace/comma/tab-separated IMU TXT/CSV
  static bool loadImuFile(const std::string &filepath,
                          std::vector<lidar_utils::ImuSample> &imu_list);

  // Loads Odometry velocity CSV (timestamp, vx [, vy, vz])
  static bool loadOdomFile(const std::string &filepath, double odom_scale,
                           std::vector<lidar_utils::OdomSample> &odom_list);

  // Scans folder for .pcd/.las files and parses millisecond timestamps from filenames
  static bool scanLidarFolder(const std::string &folder_path, double time_offset,
                              std::vector<lidar_utils::LidarContent> &file_list);

  // Reads point cloud and per-point timestamps from file
  static bool readPointCloud(const std::string &filepath,
                             lidar_utils::FileFormat format,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                             std::vector<double> &timestamps);
};

} // namespace lidar_dynamic_merge

