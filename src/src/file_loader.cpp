#include "lidar_dynamicmerge_c/file_loader.hpp"
#include "lidar_dynamicmerge_c/gnss_processor.hpp"
#include "lidar_dynamicmerge_c/logger.hpp"
#include <lidar_utils/cloud_utils.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace lidar_dynamic_merge {

bool FileLoader::loadGnssCSV(const std::string &filepath,
                             std::vector<GNSSData> &gnss_list,
                             Eigen::Vector3d &local_origin) {
  processGNSSData(filepath, gnss_list, local_origin);

  auto by_time = [](const GNSSData &a, const GNSSData &b) {
    return a.timestamp < b.timestamp;
  };
  if (!std::is_sorted(gnss_list.begin(), gnss_list.end(), by_time)) {
    LOG_WARN("FileLoader: GNSS records are not chronological; sorting them by timestamp.");
    std::sort(gnss_list.begin(), gnss_list.end(), by_time);
  }
  return !gnss_list.empty();
}

bool FileLoader::loadImuFile(const std::string &filepath,
                             std::vector<lidar_utils::ImuSample> &imu_list) {
  if (filepath.empty()) {
    LOG_WARN("FileLoader: No imu_file configured; rotation compensation will be disabled.");
    return false;
  }

  LOG_INFO("FileLoader: Loading IMU File: " << filepath);
  std::ifstream imu_file(filepath);
  if (!imu_file.is_open()) {
    LOG_FATAL("FileLoader: Could not open the IMU file: " << filepath);
    exit(1);
  }

  std::string line, word;
  while (std::getline(imu_file, line)) {
    for (char &c : line)
      if (c == ',' || c == '\t')
        c = ' ';

    std::vector<std::string> row;
    std::stringstream ss(line);
    while (ss >> word)
      row.push_back(word);

    if (row.size() < 4)
      continue;

    try {
      lidar_utils::ImuSample s;
      s.time = std::stod(row[0]);
      s.gyroX = std::stod(row[1]);
      s.gyroY = std::stod(row[2]);
      s.gyroZ = std::stod(row[3]);
      if (row.size() >= 7) {
        s.accX = std::stod(row[4]);
        s.accY = std::stod(row[5]);
        s.accZ = std::stod(row[6]);
      }
      imu_list.push_back(s);
    } catch (...) {
      // Ignore header or malformed rows
    }
  }
  imu_file.close();

  std::sort(imu_list.begin(), imu_list.end(),
            [](const lidar_utils::ImuSample &a, const lidar_utils::ImuSample &b) {
              return a.time < b.time;
            });

  if (imu_list.empty()) {
    LOG_ERROR("FileLoader: Parsed 0 IMU records from " << filepath);
    return false;
  }

  LOG_INFO("FileLoader: Successfully loaded "
           << imu_list.size() << " IMU records, time range ["
           << std::setprecision(13) << imu_list.front().time << ", "
           << imu_list.back().time << "].");
  return true;
}

bool FileLoader::loadOdomFile(const std::string &filepath, double odom_scale,
                              std::vector<lidar_utils::OdomSample> &odom_list) {
  if (filepath.empty()) {
    LOG_WARN("FileLoader: No odom_file configured; ODOM_TRANS translation will be unavailable.");
    return false;
  }

  LOG_INFO("FileLoader: Loading Odom File: " << filepath);
  std::ifstream odom_file(filepath);
  if (!odom_file.is_open()) {
    LOG_FATAL("FileLoader: Could not open the Odom file: " << filepath);
    exit(1);
  }

  std::string line, word;
  while (std::getline(odom_file, line)) {
    for (char &c : line)
      if (c == ',' || c == '\t')
        c = ' ';

    std::vector<std::string> row;
    std::stringstream ss(line);
    while (ss >> word)
      row.push_back(word);

    if (row.size() < 2)
      continue;

    try {
      lidar_utils::OdomSample s;
      s.time = std::stod(row[0]);
      s.vx = std::stod(row[1]) * odom_scale;
      if (row.size() >= 3)
        s.vy = std::stod(row[2]) * odom_scale;
      if (row.size() >= 4)
        s.vz = std::stod(row[3]) * odom_scale;
      odom_list.push_back(s);
    } catch (...) {
      // Ignore header or malformed rows
    }
  }
  odom_file.close();

  std::sort(odom_list.begin(), odom_list.end(),
            [](const lidar_utils::OdomSample &a, const lidar_utils::OdomSample &b) {
              return a.time < b.time;
            });

  if (odom_list.empty()) {
    LOG_ERROR("FileLoader: Parsed 0 odom records from " << filepath);
    return false;
  }

  double vmax = 0.0;
  for (const auto &s : odom_list)
    vmax = std::max(vmax, std::abs(s.vx));
  LOG_INFO("FileLoader: Successfully loaded "
           << odom_list.size() << " odom records (scale " << odom_scale
           << ", max |vx| " << vmax << " m/s, time range ["
           << std::setprecision(13) << odom_list.front().time << ", "
           << odom_list.back().time << "]).");
  return true;
}

bool FileLoader::scanLidarFolder(const std::string &folder_path, double time_offset,
                                 std::vector<lidar_utils::LidarContent> &file_list) {
  LOG_INFO("FileLoader: Scanning folder: " << folder_path);
  lidar_utils::CloudUtils::readContent(folder_path, file_list);

  if (time_offset != 0.0) {
    for (auto &entry : file_list)
      entry.timestamp += time_offset;
  }

  if (file_list.empty()) {
    LOG_ERROR("FileLoader: Found 0 frames in " << folder_path);
    return false;
  }

  LOG_INFO("FileLoader: Successfully indexed " << file_list.size() << " frames from " << folder_path);
  return true;
}

bool FileLoader::readPointCloud(const std::string &filepath,
                                lidar_utils::FileFormat format,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                std::vector<double> &timestamps) {
  std::string fp = filepath;
  lidar_utils::CloudUtils::readFile(cloud, timestamps, fp, format);
  return !cloud->empty();
}

} // namespace lidar_dynamic_merge

