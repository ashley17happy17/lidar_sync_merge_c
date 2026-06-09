#include "lidar_dynamic_merge.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace lidar_dynamic_merge {

LiDARDynamicMerge::LiDARDynamicMerge(const std::string &config_file) {
  loadConfig(config_file);
  loadGnssData();
  loadLidarContent();
}

void LiDARDynamicMerge::loadConfig(const std::string &config_file) {
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
        std::cout << "[INFO] Skipping lidar " << i << " because fp is empty."
                  << std::endl;
        continue;
      }

      LidarConfig lc;
      lc.path = path;
      if (config_[prefix + "type"])
        lc.type = config_[prefix + "type"].as<std::string>();
      else
        lc.type = "UNKNOWN";
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
    std::cerr << "[ERROR] All lidar file paths are empty. Exiting."
              << std::endl;
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

void LiDARDynamicMerge::loadGnssData() {
  std::cout << "[INFO] Loading GNSS data: " << gnss_file_ << std::endl;
  std::ifstream gnss_file(gnss_file_);

  if (!gnss_file.is_open()) {
    std::cerr << "[ERROR] Could not open the file." << std::endl;
    exit(1);
  }

  std::string line, word;

  // 2. Read line by line
  bool origin_set = false;
  double lat0 = 0.0, lon0 = 0.0;
  const double R = 6378137.0; // Earth equatorial radius in meters

  while (std::getline(gnss_file, line)) {
    std::vector<std::string> row;
    std::stringstream ss(line);

    // 3. Split individual line by comma
    while (std::getline(ss, word, ',')) {
      row.push_back(word);
    }

    if (row.size() < 11)
      continue;

    try {
      // Extract raw Lat/Lon/H
      double lat = std::stod(row[1]);
      double lon = std::stod(row[2]);
      double h = std::stod(row[3]);

      // Set the first valid point as the local origin
      if (!origin_set) {
        lat0 = lat;
        lon0 = lon;
        origin_set = true;
      }

      // Convert WGS84 to Local ENU (Flat Earth Approximation)
      double lat_rad = lat * M_PI / 180.0;
      double lon_rad = lon * M_PI / 180.0;
      double lat0_rad = lat0 * M_PI / 180.0;
      double lon0_rad = lon0 * M_PI / 180.0;

      double x = R * std::cos(lat0_rad) * (lon_rad - lon0_rad); // East
      double y = R * (lat_rad - lat0_rad);                      // North
      double z = h;                                             // Up

      // Keep RPY in degrees as expected by motion_impl
      double roll = std::stod(row[7]);
      double pitch = std::stod(row[8]);

      // Heading: input is North=0, East=90 (Clockwise).
      // Standard ENU Yaw is East=0, North=90 (Counter-Clockwise).
      double heading_deg = std::stod(row[9]);
      double yaw = 90.0 - heading_deg;

      // Normalize yaw to [-180, 180]
      while (yaw > 180.0)
        yaw -= 360.0;
      while (yaw < -180.0)
        yaw += 360.0;

      GNSSData data;
      data.timestamp = std::stod(row[0]);
      data.pose_6d = Eigen::VectorXd(6);
      data.pose_6d << x, y, z, roll, pitch, yaw;
      data.position_std = Eigen::Vector3d(std::stod(row[4]), std::stod(row[5]),
                                          std::stod(row[6]));
      data.geoidSep = std::stod(row[10]);
      gnss_data_list_.push_back(data);
    } catch (...) {
      // Ignore header or malformed rows
    }
  }
  gnss_file.close();
  std::cout << "[INFO] Successfully loaded " << gnss_data_list_.size()
            << " GNSS records.\n";
}

void LiDARDynamicMerge::loadLidarContent() {
  for (const auto &[id, config] : lidars_) {
    std::cout << "[INFO] Loading file list for lidar " << id << ": "
              << config.path << std::endl;

    std::vector<lidar_utils::LidarContent> file_list;
    lidar_utils::CloudUtils::readContent(config.path, file_list);

    lidar_files_[id] = file_list;
    std::cout << "[INFO] Lidar " << id << " successfully loaded "
              << file_list.size() << " frames." << std::endl;
  }
}

void LiDARDynamicMerge::processFrame(
    const GNSSData &gnss, const GNSSData *next_gnss,
    const std::map<int, std::string> &matched_files) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  // For each lidar, find nearest timestamp, load, process, transform, merge
  for (const auto &[id, config] : lidars_) {
    std::string matched_file = matched_files.at(id);
    std::vector<double> timestamps;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    // === Read LiDAR File ===
    lidar_utils::CloudUtils::readFile(cloud, timestamps, matched_file,
                                      config.format);

    // === Crop Cloud ===
    Eigen::Vector3f minBound(crop_min_[0], crop_min_[1], crop_min_[2]);
    Eigen::Vector3f maxBound(crop_max_[0], crop_max_[1], crop_max_[2]);
    lidar_utils::CloudUtils::cropCloud(cloud, timestamps, minBound, maxBound);

    // === Denoise Cloud ===
    lidar_utils::CloudUtils::denoiseCloud(cloud, 0.01f, 0.1f);

    // === Remove Artifact Cloud ===
    lidar_utils::CloudUtils::removeArtifactCloud(cloud, timestamps);

    // === Transform LiDAR to GNSS Antenna Frame ===
    Eigen::Matrix4d ext = lidar_utils::CloudUtils::getExtrinsics(
        config.type, config.trans, config.rot);
    lidar_utils::CloudUtils::directGeoreference(cloud, ext);

    // === Motion Compensation and Transform to Global Frame ===
    // Interpolate between current GNSS and NEXT GNSS
    Eigen::VectorXd posCurr(7);
    posCurr << gnss.timestamp, gnss.pose_6d;
    Eigen::VectorXd posNext(7);
    posNext << next_gnss->timestamp, next_gnss->pose_6d;

    lidar_utils::CloudUtils::motionCompensateAndDG(cloud, timestamps, posCurr,
                                                   posNext, motion_enable_);

    // === Merge Cloud ===
    lidar_utils::CloudUtils::mergeCloud(merged_cloud, cloud, merge_voxel_size_);

    /*
    // --------------------------DEBUG TEST START-----------------------
    std::string out_file_tmp = out_fp_ + "/" +
                               std::to_string(static_cast<uint64_t>(
                                   std::round(gnss.timestamp * 1e3))) +
                               "_" + std::to_string(id) + ".pcd";
    lidar_utils::CloudUtils::saveFile(cloud, out_file_tmp, out_format_);
    // --------------------------DEBUG TEST END-----------------------
    */
  }

  if (lidars_.count(out_lidar_)) {
    // === Transform from Global Frame to GNSS Frame ===
    Eigen::Matrix4d predicted_pose_inv = gnss.getTransform().inverse();
    lidar_utils::CloudUtils::directGeoreference(merged_cloud,
                                                predicted_pose_inv);

    // === Transform from GNSS Frame to out_lidar Frame ===
    Eigen::Matrix4d inv_ext;
    inv_ext = lidar_utils::CloudUtils::getExtrinsics(lidars_[out_lidar_].type,
                                                     lidars_[out_lidar_].trans,
                                                     lidars_[out_lidar_].rot)
                  .inverse();
    lidar_utils::CloudUtils::directGeoreference(merged_cloud, inv_ext);

    // === Save Dynamic Merge File ===
    std::string out_file = out_fp_ + "/" +
                           std::to_string(static_cast<uint64_t>(
                               std::round(gnss.timestamp * 1e3))) +
                           ".pcd";
    lidar_utils::CloudUtils::saveFile(merged_cloud, out_file, out_format_);
  }
}

void LiDARDynamicMerge::run() {
  std::cout << "[INFO] Starting synchronization loop...\n"
            << "---------------------------------------------------------\n";

  // Synchronization threshold (e.g., 0.05 seconds = 50 ms)
  double sync_threshold = 1.0 / gnss_freq_ / 2;
  int processed_frames = 0;

  // Enforce GNSS frequency downsampling (e.g. 10Hz data -> 1Hz processing)
  double process_interval = 1.0 / gnss_freq_ * 0.9;
  double last_processed_time = -1.0;

  // Track the last matched index to optimize searching sequentially
  std::map<int, size_t> search_start_idx;
  for (const auto &[id, file_list] : lidar_files_) {
    search_start_idx[id] = 0;
  }

  GNSSData curr_gnss;
  std::map<int, std::string> curr_matched_files;
  bool has_curr = false;

  for (const auto &gnss : gnss_data_list_) {
    // 1. Reject bad GNSS data based on threshold
    if (gnss.position_std.norm() > gnss_std_thres_) {
      continue;
    }

    double target_timestamp = gnss.timestamp;

    // Throttle GNSS processing based on gnss_freq parameter
    if (last_processed_time > 0 &&
        (target_timestamp - last_processed_time) < process_interval) {
      continue;
    }

    last_processed_time = target_timestamp;
    bool all_matched = true;
    std::map<int, std::string> matched_files;

    // Check if ALL lidars have a frame within the sync_threshold
    for (const auto &[id, file_list] : lidar_files_) {
      if (file_list.empty()) {
        all_matched = false;
        break;
      }

      size_t start_idx = search_start_idx[id];

      // Binary search starting from the last matched index
      auto it = std::lower_bound(file_list.begin() + start_idx, file_list.end(),
                                 target_timestamp,
                                 [](const lidar_utils::LidarContent &a,
                                    double val) { return a.timestamp < val; });

      double min_diff = 1e9;
      std::string matched_file;
      size_t matched_idx = start_idx;

      // Find the Nearest LiDAR Timestamp To GNSS Timestamp
      if (it == file_list.end()) {
        auto prev = it - 1;
        min_diff = std::abs(prev->timestamp - target_timestamp);
        matched_file = prev->filename;
        matched_idx = std::distance(file_list.begin(), prev);
      } else if (it == file_list.begin()) {
        min_diff = std::abs(it->timestamp - target_timestamp);
        matched_file = it->filename;
        matched_idx = std::distance(file_list.begin(), it);
      } else {
        auto prev = it - 1;
        double diff_prev = std::abs(prev->timestamp - target_timestamp);
        double diff_it = std::abs(it->timestamp - target_timestamp);
        if (diff_prev < diff_it) {
          min_diff = diff_prev;
          matched_file = prev->filename;
          matched_idx = std::distance(file_list.begin(), prev);
        } else {
          min_diff = diff_it;
          matched_file = it->filename;
          matched_idx = std::distance(file_list.begin(), it);
        }
      }

      if (min_diff > sync_threshold) {
        all_matched = false;
        break;
      }

      // Cache index to start from here next time
      search_start_idx[id] = matched_idx;
      matched_files[id] = matched_file;
    }

    if (all_matched) {
      if (has_curr) {
        // Process only if we have a NEXT frame for motion compensation
        processFrame(curr_gnss, &gnss, curr_matched_files);
        processed_frames++;
        std::cout << "[INFO] Finish merge GNSS timestamp:"
                  << std::setprecision(13) << target_timestamp
                  << ", frame: " << processed_frames << std::endl;
      }
      curr_gnss = gnss;
      curr_matched_files = matched_files;
      has_curr = true;
    }
  }

  std::cout << "[INFO] LiDAR_Dynamic_Merge finished running. Processed "
            << processed_frames << " synchronized frames.\n";
}

} // namespace lidar_dynamic_merge