#include "lidar_dynamicmerge_c/dynamic_merge_node.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace lidar_dynamic_merge {

DynamicMergeNode::DynamicMergeNode(const std::string &config_file) {
  loadConfig(config_file);
  loadGnssData();
  loadImuData();
  loadOdomData();
  loadLidarContent();
}

void DynamicMergeNode::loadGnssData() {
  Eigen::Vector3d local_origin;
  processGNSSData(gnss_file_, gnss_data_list_, local_origin);

  // Pose interpolation, frame synchronisation and the library's GNSS deskew all
  // assume a chronological trajectory.
  auto by_time = [](const GNSSData &a, const GNSSData &b) {
    return a.timestamp < b.timestamp;
  };
  if (!std::is_sorted(gnss_data_list_.begin(), gnss_data_list_.end(), by_time)) {
    LOG_WARN("Load GNSS: GNSS records are not chronological; sorting them by "
             "timestamp.");
    std::sort(gnss_data_list_.begin(), gnss_data_list_.end(), by_time);
  }

  // Build a position-only view of the trajectory (in local TWD97 world frame)
  // used as the translation source for motion compensation.
  gnss_samples_.clear();
  gnss_samples_.reserve(gnss_data_list_.size());
  for (const auto &g : gnss_data_list_) {
    if (g.pose_6d.size() < 3)
      continue;
    lidar_utils::GnssSample s;
    s.time = g.timestamp;
    s.x = g.pose_6d[0];
    s.y = g.pose_6d[1];
    s.z = g.pose_6d[2];
    gnss_samples_.push_back(s);
  }
}

void DynamicMergeNode::loadImuData() {
  if (imu_file_.empty()) {
    LOG_WARN("Load IMU: No imu_file configured; rotation compensation will be "
             "disabled (translation-only deskew).");
    return;
  }

  LOG_INFO("Load IMU: Loading IMU File: " << imu_file_);
  std::ifstream imu_file(imu_file_);
  if (!imu_file.is_open()) {
    LOG_FATAL("Load IMU: Could not open the IMU file: " << imu_file_);
    exit(1);
  }

  std::string line, word;
  while (std::getline(imu_file, line)) {
    // Accept comma-, tab- or space-separated rows: normalise the separators
    // first, then split on whitespace.
    for (char &c : line)
      if (c == ',' || c == '\t')
        c = ' ';

    std::vector<std::string> row;
    std::stringstream ss(line);
    while (ss >> word)
      row.push_back(word);

    // Expected columns: timestamp[s], gyro_x, gyro_y, gyro_z [rad/s], and
    // optionally acc_x, acc_y, acc_z [m/s^2] in columns 4-6 (gravity-removed
    // "free acceleration", only used by IMU_ACC). Gyro/accel are assumed to be
    // in the GNSS/vehicle (FLU) frame already; no axis correction is applied.
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
      imu_data_list_.push_back(s);
    } catch (...) {
      // Ignore header or malformed rows
    }
  }
  imu_file.close();

  // The library integrates/interpolates assuming chronological order.
  std::sort(imu_data_list_.begin(), imu_data_list_.end(),
            [](const lidar_utils::ImuSample &a,
               const lidar_utils::ImuSample &b) { return a.time < b.time; });

  if (imu_data_list_.empty()) {
    LOG_ERROR("Load IMU: Parsed 0 IMU records from "
              << imu_file_
              << "! Expected rows of 'timestamp gyroX gyroY gyroZ' separated by "
                 "comma, tab or space. Rotation compensation will be disabled.");
    return;
  }

  LOG_INFO("Load IMU: Successfully loaded "
           << imu_data_list_.size() << " IMU records, time range ["
           << std::setprecision(13) << imu_data_list_.front().time << ", "
           << imu_data_list_.back().time << "].");
}

void DynamicMergeNode::loadOdomData() {
  if (odom_file_.empty()) {
    LOG_WARN("Load Odom: No odom_file configured; ODOM_TRANS translation will "
             "be unavailable.");
    return;
  }

  LOG_INFO("Load Odom: Loading Odom File: " << odom_file_);
  std::ifstream odom_file(odom_file_);
  if (!odom_file.is_open()) {
    LOG_FATAL("Load Odom: Could not open the Odom file: " << odom_file_);
    exit(1);
  }

  std::string line, word;
  while (std::getline(odom_file, line)) {
    // Accept comma-, tab- or space-separated rows.
    for (char &c : line)
      if (c == ',' || c == '\t')
        c = ' ';

    std::vector<std::string> row;
    std::stringstream ss(line);
    while (ss >> word)
      row.push_back(word);

    // Expected columns: timestamp[s], vx [, vy, vz] [m/s], velocity in the
    // vehicle (FLU) frame. A forward-only wheel odometer needs only vx.
    if (row.size() < 2)
      continue;

    try {
      lidar_utils::OdomSample s;
      s.time = std::stod(row[0]);
      // odom_scale_ converts the file's unit to the m/s the library expects.
      s.vx = std::stod(row[1]) * odom_scale_;
      if (row.size() >= 3)
        s.vy = std::stod(row[2]) * odom_scale_;
      if (row.size() >= 4)
        s.vz = std::stod(row[3]) * odom_scale_;
      odom_data_list_.push_back(s);
    } catch (...) {
      // Ignore header or malformed rows
    }
  }
  odom_file.close();

  // The library integrates assuming chronological order.
  std::sort(odom_data_list_.begin(), odom_data_list_.end(),
            [](const lidar_utils::OdomSample &a,
               const lidar_utils::OdomSample &b) { return a.time < b.time; });

  if (odom_data_list_.empty()) {
    LOG_ERROR("Load Odom: Parsed 0 odom records from "
              << odom_file_
              << "! Expected rows of 'timestamp vx [vy vz]'. ODOM_TRANS will be "
                 "unavailable.");
    return;
  }

  // Report the speed range actually handed to the deskew, so a unit mistake is
  // visible in the log instead of silently scaling every translation.
  double vmax = 0.0;
  for (const auto &s : odom_data_list_)
    vmax = std::max(vmax, std::abs(s.vx));
  LOG_INFO("Load Odom: Successfully loaded "
           << odom_data_list_.size() << " odom records (scale " << odom_scale_
           << ", max |vx| " << vmax << " m/s, time range ["
           << std::setprecision(13) << odom_data_list_.front().time << ", "
           << odom_data_list_.back().time << "]).");
}

void DynamicMergeNode::loadLidarContent() {
  for (const auto &[id, config] : lidars_) {
    LOG_INFO("Load LiDAR: Loading file list for LiDAR " << id << ": "
                                                        << config.path);

    std::vector<lidar_utils::LidarContent> file_list;
    lidar_utils::CloudUtils::readContent(config.path, file_list);

    // Put the filename-derived times on the GNSS clock before they reach
    // synchronizeFrames. The file names themselves are untouched; only the
    // timestamps this run works with are shifted.
    if (config.time_offset != 0.0)
      for (auto &entry : file_list)
        entry.timestamp += config.time_offset;

    if (file_list.empty()) {
      LOG_ERROR("Load LiDAR: LiDAR "
                << id << " loaded 0 frames from " << config.path
                << "! Please check if the directory is correct and "
                   "contains valid files.");
    }

    lidar_files_[id] = file_list;
    LOG_INFO("Load LiDAR: LiDAR " << id << " successfully loaded "
                                  << file_list.size() << " frames.");
  }
}

Eigen::Matrix4d DynamicMergeNode::interpolatePoseAt(double t) const {
  if (gnss_data_list_.empty())
    return Eigen::Matrix4d::Identity();

  auto it = std::lower_bound(
      gnss_data_list_.begin(), gnss_data_list_.end(), t,
      [](const GNSSData &a, double val) { return a.timestamp < val; });

  // Outside the trajectory: clamp to the nearest end rather than extrapolate.
  if (it == gnss_data_list_.begin())
    return gnss_data_list_.front().getTransform();
  if (it == gnss_data_list_.end())
    return gnss_data_list_.back().getTransform();

  const GNSSData &prev = *(it - 1);
  const GNSSData &next = *it;
  const double denom = next.timestamp - prev.timestamp;
  const double ratio = denom > 0.0 ? (t - prev.timestamp) / denom : 0.0;

  const Eigen::Matrix4d T_prev = prev.getTransform();
  const Eigen::Matrix4d T_next = next.getTransform();

  // Slerp on the rotation (takes the shortest path, so heading wrap-around is
  // handled) and lerp on the position.
  const Eigen::Quaterniond q_prev(T_prev.block<3, 3>(0, 0));
  const Eigen::Quaterniond q_next(T_next.block<3, 3>(0, 0));

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q_prev.slerp(ratio, q_next).normalized().matrix();
  T.block<3, 1>(0, 3) = (1.0 - ratio) * T_prev.block<3, 1>(0, 3) +
                        ratio * T_next.block<3, 1>(0, 3);
  return T;
}

void DynamicMergeNode::processFrame(
    const GNSSData &gnss, const GNSSData &next_gnss,
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

    // Same clock correction as the file list: the per-point times drive the
    // deskew and the scan-start pose lookup, so they must be on the GNSS clock
    // too, otherwise the IMU/GNSS/odom are sampled at the wrong instant.
    if (config.time_offset != 0.0)
      for (double &ts : timestamps)
        ts += config.time_offset;

    if (cloud->empty()) {
      LOG_WARN("Process Frame: LiDAR "
               << id
               << " loaded an empty point cloud from file: " << matched_file);
      continue;
    }

    // === Crop Cloud ===
    Eigen::Vector3f minBound(crop_min_[0], crop_min_[1], crop_min_[2]);
    Eigen::Vector3f maxBound(crop_max_[0], crop_max_[1], crop_max_[2]);
    lidar_utils::CloudUtils::cropCloud(cloud, timestamps, minBound, maxBound);

    // === Denoise Cloud ===
    lidar_utils::CloudUtils::denoiseCloud(cloud, denoise_radius_,
                                          denoise_epsilon_);

    // === Remove Artifact Cloud ===
    lidar_utils::CloudUtils::removeArtifactCloud(cloud, timestamps);

    // === Transform LiDAR to GNSS Antenna Frame ===
    Eigen::Matrix4d ext = lidar_utils::CloudUtils::getExtrinsics(
        config.type, config.trans, config.rot);
    lidar_utils::CloudUtils::directGeoreference(cloud, cloud, ext);

    // === Motion Compensation (deskew) + Transform to Global Frame ===
    // Deskew brings the scan to its scan-start pose, then it is georeferenced
    // into the local (TWD97) world frame. A LiDAR frame starts up to 1/lidar_hz
    // after the GNSS fix it was matched to, so georeference with the pose
    // interpolated at the scan-start epoch, otherwise the whole cloud is planted
    // where the vehicle was a fraction of a second earlier (~0.4 m at 8 m/s).
    // Rotation always comes from the gyro; translation from motion_method_
    // (GNSS / ODOM / IMU_ACC). The fused call derives R_vehicle_from_world and,
    // for IMU_ACC, the scan-start velocity seed from the GNSS trajectory.
    if (use_legacy_motion_) {
      // Legacy path: no IMU at all. Each point gets an absolute pose slerped
      // between the current and next GNSS fix, which also georeferences it.
      // Kept so the newer methods can be compared against it; note it clamps
      // the part of the scan that runs past next_gnss and anchors the cloud at
      // gnss.timestamp rather than at the scan start.
      Eigen::VectorXd posCurr(7), posNext(7);
      posCurr << gnss.timestamp, gnss.pose_6d;
      posNext << next_gnss.timestamp, next_gnss.pose_6d;
      lidar_utils::CloudUtils::motionCompensateAndDG(cloud, timestamps, posCurr,
                                                     posNext, motion_enable_);
    } else {
      const double scan_start =
          timestamps.empty()
              ? gnss.timestamp
              : *std::min_element(timestamps.begin(), timestamps.end());
      const Eigen::Matrix4d T_vehicle_to_world = interpolatePoseAt(scan_start);
      lidar_utils::CloudUtils::motionCompensateAndDG(
          cloud, timestamps, imu_data_list_, motion_method_, T_vehicle_to_world,
          gnss_samples_, odom_data_list_, Eigen::Vector3d::Zero(),
          motion_enable_);
    }

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
    if (merged_cloud->empty()) {
      LOG_WARN(
          "Process Frame: Merged point cloud is empty, skipping save for GNSS "
          "timestamp: "
          << std::setprecision(13) << gnss.timestamp);
      return;
    }

    // === Transform from Global Frame to GNSS Frame ===
    Eigen::Matrix4d predicted_pose_inv = gnss.getTransform().inverse();
    lidar_utils::CloudUtils::directGeoreference(merged_cloud, merged_cloud,
                                                predicted_pose_inv);

    // === Transform from GNSS Frame to Target Frame ===
    Eigen::Matrix4d inv_ext;
    if (out_la_.isZero() && out_bs_.isZero()) {
      inv_ext = lidar_utils::CloudUtils::getExtrinsics(lidars_[out_lidar_].type,
                                                       lidars_[out_lidar_].trans,
                                                       lidars_[out_lidar_].rot)
                    .inverse();
    } else {
      // Convert FRD Lever Arm to FLU (X_flu = X_frd, Y_flu = -Y_frd, Z_flu = -Z_frd)
      Eigen::Vector3d out_la_flu(out_la_.x(), -out_la_.y(), -out_la_.z());
      inv_ext = lidar_utils::CloudUtils::transformEOP(Eigen::Matrix4d::Identity(),
                                                      out_la_flu, out_bs_)
                    .inverse();
    }
    lidar_utils::CloudUtils::directGeoreference(merged_cloud, merged_cloud,
                                                inv_ext);

    // === Save Dynamic Merge File ===
    std::string out_file = out_fp_ + "/" +
                           std::to_string(static_cast<uint64_t>(
                               std::round(gnss.timestamp * 1e3))) +
                           ".pcd";
    lidar_utils::CloudUtils::saveFile(merged_cloud, out_file, out_format_);
  }
}

void DynamicMergeNode::run() {
  LOG_INFO(
      "Run: Starting synchronization "
      "loop...\n---------------------------------------------------------");

  auto sync_frames = synchronizeFrames(gnss_data_list_, lidar_files_,
                                       gnss_freq_, gnss_std_thres_, lidar_hz_);
  int processed_frames = 0;

#pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < sync_frames.size(); ++i) {
    const auto &frame = sync_frames[i];
    processFrame(frame.curr_gnss, frame.next_gnss, frame.matched_files);
    
    int current_processed;
#pragma omp atomic capture
    {
      processed_frames++;
      current_processed = processed_frames;
    }
    
    LOG_INFO("Run: Finish merge GNSS timestamp: "
             << std::setprecision(13) << frame.next_gnss.timestamp
             << ", frame: " << current_processed << " / " << sync_frames.size());
  }

  LOG_INFO("Run: LiDAR_Dynamic_Merge finished running. Processed "
           << processed_frames << " synchronized frames.");
}

} // namespace lidar_dynamic_merge