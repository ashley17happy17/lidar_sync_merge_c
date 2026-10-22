#include "lidar_dynamicmerge_c/dynamic_merge_node.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace lidar_dynamic_merge {

DynamicMergeNode::DynamicMergeNode(const std::string &config_file) {
  loadConfig(config_file);
  loadData();
  buildGnssSamples();
}

void DynamicMergeNode::loadData() {
  Eigen::Vector3d local_origin;
  if (!bag_file_.empty()) {
    LOG_INFO("Load Data: Operating in ROS 1 BAG MODE using " << bag_file_);
    bag_loader_ = std::make_unique<BagLoader>(bag_file_);
    if (!bag_loader_->open()) {
      LOG_FATAL("Load Data: Could not open bag file: " << bag_file_);
      exit(1);
    }
    bag_loader_->loadTelemetry(gnss_source_, imu_source_, odom_source_, odom_scale_,
                               gnss_data_list_, local_origin, imu_data_list_, odom_data_list_);
    bag_loader_->indexAllLidarTopics(lidars_, lidar_files_);
  } else {
    LOG_INFO("Load Data: Operating in FILE MODE");
    FileLoader::loadGnssCSV(gnss_source_, gnss_data_list_, local_origin);
    FileLoader::loadImuFile(imu_source_, imu_data_list_);
    FileLoader::loadOdomFile(odom_source_, odom_scale_, odom_data_list_);
    for (const auto &[id, config] : lidars_) {
      std::vector<lidar_utils::LidarContent> list;
      FileLoader::scanLidarFolder(config.source, config.time_offset, list);
      lidar_files_[id] = list;
    }
  }
}

void DynamicMergeNode::buildGnssSamples() {
  auto by_time = [](const GNSSData &a, const GNSSData &b) {
    return a.timestamp < b.timestamp;
  };
  if (!std::is_sorted(gnss_data_list_.begin(), gnss_data_list_.end(), by_time)) {
    LOG_WARN("DynamicMergeNode: GNSS records are not chronological; sorting them by timestamp.");
    std::sort(gnss_data_list_.begin(), gnss_data_list_.end(), by_time);
  }

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
    const std::map<int, std::string> &matched_files,
    const std::string &time_stem) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  // For each lidar, find nearest timestamp, load, process, transform, merge
  for (const auto &[id, config] : lidars_) {
    std::string matched_file = matched_files.at(id);
    std::vector<double> timestamps;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    // === Read LiDAR File / Message ===
    if (bag_loader_) {
      size_t msg_idx = std::stoull(matched_file);
      bag_loader_->readPointCloud(id, msg_idx, config.type, cloud, timestamps);
    } else {
      FileLoader::readPointCloud(matched_file, config.format, cloud, timestamps);
    }

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
    std::string out_file_tmp = out_source_ + "/" +
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
    std::string stem = time_stem.empty() ? computeTimeStem(gnss, matched_files) : time_stem;
    std::string out_file = (std::filesystem::path(out_source_) / (stem + ".pcd")).string();
    lidar_utils::CloudUtils::saveFile(merged_cloud, out_file, out_format_);
  }
}

void DynamicMergeNode::runBagPipelined(std::vector<SyncFrame> &sync_frames) {
  const size_t total_frames = sync_frames.size();
  BoundedQueue<FramePayload> queue(32);
  std::atomic<int> processed_frames{0};

  // Launch sequential producer thread (reads rosbag sequentially from start to finish)
  std::thread producer([&]() {
    std::string last_pushed_stem;
    for (size_t i = 0; i < total_frames; ++i) {
      FramePayload payload;
      payload.frame_index = i;
      payload.curr_gnss = sync_frames[i].curr_gnss;
      payload.next_gnss = sync_frames[i].next_gnss;

      for (const auto &[id, config] : lidars_) {
        auto it = sync_frames[i].matched_files.find(id);
        if (it != sync_frames[i].matched_files.end()) {
          size_t msg_idx = std::stoull(it->second);
          auto msg = bag_loader_->readPointCloud2Msg(id, msg_idx);
          if (msg) {
            payload.lidar_msgs[id] = msg;
          }
        }
      }

      int ref_id = lidars_.count(1) ? 1 : (lidars_.count(out_lidar_) ? out_lidar_ : (lidars_.empty() ? -1 : lidars_.begin()->first));
      if (ref_id < 0 || payload.lidar_msgs.find(ref_id) == payload.lidar_msgs.end() || !payload.lidar_msgs.at(ref_id)) {
        for (const auto &[id, msg] : payload.lidar_msgs) {
          if (msg) {
            ref_id = id;
            break;
          }
        }
      }

      std::string out_stem;
      double t_lidar = payload.curr_gnss.timestamp;
      if (ref_id >= 0 && payload.lidar_msgs.count(ref_id) && payload.lidar_msgs.at(ref_id)) {
        const auto &msg = payload.lidar_msgs.at(ref_id);
        if (msg->header.stamp.sec > 0 || msg->header.stamp.nsec > 0) {
          t_lidar = static_cast<double>(msg->header.stamp.sec) +
                    static_cast<double>(msg->header.stamp.nsec) * 1e-9;
          uint64_t ts_ms = static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL +
                           static_cast<uint64_t>(std::round(msg->header.stamp.nsec * 1e-6));
          out_stem = std::to_string(ts_ms);
        }
      }

      if (out_stem.empty()) {
        out_stem = std::to_string(static_cast<uint64_t>(std::round(payload.curr_gnss.timestamp * 1e3)));
      }

      payload.time_stem = out_stem;
      payload.lidar_time = t_lidar;
      sync_frames[i].time_stem = out_stem;
      sync_frames[i].lidar_time = t_lidar;

      if (!last_pushed_stem.empty() && out_stem == last_pushed_stem) {
        LOG_WARN("runBagPipelined: Skipping duplicate frame with time_stem: " << out_stem);
        continue;
      }
      last_pushed_stem = out_stem;

      queue.push(std::move(payload));
    }
    queue.finish();
  });

  // Parallel consumer threads (unpack, crop, denoise, deskew, georeference, merge, save)
#pragma omp parallel
  {
    FramePayload payload;
    while (queue.pop(payload)) {
      processFrameFromPayload(payload);
      int current_processed = ++processed_frames;
      LOG_INFO("Run: Finish merge frame " << current_processed << " / " << total_frames
               << " (GNSS timestamp: " << std::setprecision(13) << payload.curr_gnss.timestamp << ")");
    }
  }

  if (producer.joinable()) {
    producer.join();
  }

  LOG_INFO("Run: LiDAR_Dynamic_Merge finished running. Processed "
           << processed_frames << " synchronized frames.");
}

void DynamicMergeNode::processFrameFromPayload(const FramePayload &payload) {
  const auto &gnss = payload.curr_gnss;
  const auto &next_gnss = payload.next_gnss;
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (const auto &[id, config] : lidars_) {
    auto it = payload.lidar_msgs.find(id);
    if (it == payload.lidar_msgs.end() || !it->second) {
      continue;
    }

    std::vector<double> timestamps;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    if (!BagLoader::unpackPointCloud2(it->second, config.type, cloud, timestamps, config.time_offset)) {
      continue;
    }

    if (cloud->empty()) {
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
    if (use_legacy_motion_) {
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
    std::string stem = payload.time_stem.empty()
                           ? std::to_string(static_cast<uint64_t>(std::round(gnss.timestamp * 1e3)))
                           : payload.time_stem;
    std::string out_file = (std::filesystem::path(out_source_) / (stem + ".pcd")).string();
    lidar_utils::CloudUtils::saveFile(merged_cloud, out_file, out_format_);
  }
}

std::string DynamicMergeNode::computeTimeStem(
    const GNSSData &gnss,
    const std::map<int, std::string> &matched_files,
    double *out_time_sec) const {
  int ref_id = lidars_.count(1) ? 1 : (lidars_.count(out_lidar_) ? out_lidar_ : (lidars_.empty() ? -1 : lidars_.begin()->first));
  if (ref_id < 0 || matched_files.find(ref_id) == matched_files.end()) {
    for (const auto &[id, path] : matched_files) {
      ref_id = id;
      break;
    }
  }

  std::string out_stem;
  double t_sec = gnss.timestamp;

  if (ref_id >= 0 && matched_files.count(ref_id)) {
    const std::string &ref_file = matched_files.at(ref_id);
    if (bag_loader_) {
      size_t msg_idx = std::stoull(ref_file);
      auto msg = bag_loader_->readPointCloud2Msg(ref_id, msg_idx);
      if (msg && (msg->header.stamp.sec > 0 || msg->header.stamp.nsec > 0)) {
        t_sec = static_cast<double>(msg->header.stamp.sec) +
                static_cast<double>(msg->header.stamp.nsec) * 1e-9;
        uint64_t ts_ms = static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL +
                         static_cast<uint64_t>(std::round(msg->header.stamp.nsec * 1e-6));
        out_stem = std::to_string(ts_ms);
      }
    } else {
      out_stem = std::filesystem::path(ref_file).stem().string();
      try {
        uint64_t ts_ms = std::stoull(out_stem);
        t_sec = static_cast<double>(ts_ms) * 1e-3;
      } catch (...) {
        t_sec = gnss.timestamp;
      }
    }
  }

  if (out_stem.empty()) {
    uint64_t ts_ms = static_cast<uint64_t>(std::round(gnss.timestamp * 1e3));
    out_stem = std::to_string(ts_ms);
    t_sec = gnss.timestamp;
  }

  if (out_time_sec) {
    *out_time_sec = t_sec;
  }
  return out_stem;
}

GNSSData DynamicMergeNode::interpolateGnss(double t, const std::vector<GNSSData> &gnss_list) const {
  return lidar_dynamic_merge::interpolateGnss(t, gnss_list);
}

void DynamicMergeNode::interpolateImuOrientation(
    double t, const std::vector<ImuOrientation> &imu_list,
    double &roll_rad, double &pitch_rad, double &yaw_rad) const {
  if (imu_list.empty()) {
    roll_rad = 0.0;
    pitch_rad = 0.0;
    yaw_rad = 0.0;
    return;
  }

  Eigen::Quaterniond q;
  if (t <= imu_list.front().timestamp) {
    q = imu_list.front().q;
  } else if (t >= imu_list.back().timestamp) {
    q = imu_list.back().q;
  } else {
    auto it = std::lower_bound(
        imu_list.begin(), imu_list.end(), t,
        [](const ImuOrientation &a, double val) { return a.timestamp < val; });
    const auto &prev = *(it - 1);
    const auto &next = *it;
    double dt = next.timestamp - prev.timestamp;
    double alpha = dt > 1e-9 ? (t - prev.timestamp) / dt : 0.0;
    q = prev.q.slerp(alpha, next.q).normalized();
  }

  Eigen::Matrix3d R = q.toRotationMatrix();
  double cy = std::sqrt(R(2, 2) * R(2, 2) + R(1, 2) * R(1, 2));
  roll_rad  = -std::atan2( R(1, 2), R(2, 2));
  pitch_rad =  std::atan2( R(0, 2), cy);
  yaw_rad   =  std::atan2(-R(0, 1), R(0, 0));
}

void DynamicMergeNode::writeEgoTrajectory(const std::vector<SyncFrame> &sync_frames) {
  if (out_traj_.empty() || sync_frames.empty()) {
    return;
  }

  LOG_INFO("Write Ego Trajectory: Writing trajectory to " << out_traj_ << " ...");
  std::ofstream out(out_traj_);
  if (!out.is_open()) {
    LOG_ERROR("Write Ego Trajectory: Failed to open output file " << out_traj_);
    return;
  }

  std::string source_val = "GNSS";
  if (gnss_source_ == "/adr" || gnss_source_ == "adr") {
    source_val = "ADR";
  } else if (gnss_source_ == "/gps/fix" || gnss_source_ == "gps/fix") {
    source_val = "DR";
  }

  out << "time,lat,lon,h,twd97x,twd97y,twd97z,roll_rad,pitch_rad,yaw_rad,source\n";

  const auto *imu_orientations = bag_loader_ ? &bag_loader_->getImuOrientations() : nullptr;
  std::string last_written_time;
  size_t written_rows = 0;

  for (const auto &frame : sync_frames) {
    double t_lidar = frame.lidar_time;
    if (t_lidar <= 0.0) {
      if (!frame.time_stem.empty()) {
        try {
          t_lidar = static_cast<double>(std::stoull(frame.time_stem)) * 1e-3;
        } catch (...) {
          t_lidar = frame.curr_gnss.timestamp;
        }
      } else {
        t_lidar = frame.curr_gnss.timestamp;
      }
    }

    std::string time_str = frame.time_stem;
    if (time_str.empty()) {
      time_str = std::to_string(static_cast<uint64_t>(std::round(t_lidar * 1e3)));
    }

    if (!last_written_time.empty() && time_str == last_written_time) {
      continue;
    }
    last_written_time = time_str;
    written_rows++;

    // 1. Interpolate position from gnss_source at t_lidar
    GNSSData interp_gnss = interpolateGnss(t_lidar, gnss_data_list_);

    // 2. Interpolate orientation at t_lidar
    double roll_rad = 0.0, pitch_rad = 0.0, yaw_rad = 0.0;
    if (bag_loader_ && imu_orientations && !imu_orientations->empty()) {
      // Bag format: use imu_source
      interpolateImuOrientation(t_lidar, *imu_orientations, roll_rad, pitch_rad, yaw_rad);
    } else {
      // Txt format (or fallback if IMU not in bag): use gnss_source
      roll_rad = interp_gnss.raw_roll * M_PI / 180.0;
      pitch_rad = interp_gnss.raw_pitch * M_PI / 180.0;
      yaw_rad = interp_gnss.raw_yaw * M_PI / 180.0;
    }

    out << time_str << ","
        << std::setprecision(16) << interp_gnss.lat << ","
        << std::setprecision(16) << interp_gnss.lon << ","
        << std::setprecision(16) << interp_gnss.alt << ","
        << std::setprecision(16) << interp_gnss.twd97_x << ","
        << std::setprecision(16) << interp_gnss.twd97_y << ","
        << std::setprecision(16) << interp_gnss.twd97_z << ","
        << std::setprecision(17) << roll_rad << ","
        << std::setprecision(17) << pitch_rad << ","
        << std::setprecision(17) << yaw_rad << ","
        << source_val << "\n";
  }
  out.close();
  LOG_INFO("Write Ego Trajectory: Successfully wrote " << written_rows << " trajectory rows to " << out_traj_);
}

void DynamicMergeNode::run() {
  LOG_INFO(
      "Run: Starting synchronization "
      "loop...\n---------------------------------------------------------");

  auto sync_frames = synchronizeFrames(gnss_data_list_, lidar_files_,
                                       gnss_freq_, gnss_std_thres_, lidar_hz_);
  if (sync_frames.empty()) {
    LOG_FATAL("Time Sync: Synchronization resulted in 0 frames! Please check GNSS frequency, std threshold, and LiDAR timestamps overlap.");
    return;
  }

  if (bag_loader_) {
    runBagPipelined(sync_frames);
    writeEgoTrajectory(sync_frames);
    return;
  }

  for (auto &frame : sync_frames) {
    frame.time_stem = computeTimeStem(frame.curr_gnss, frame.matched_files, &frame.lidar_time);
  }

  // Deduplicate any consecutive frames sharing identical time_stem
  std::vector<SyncFrame> unique_frames;
  unique_frames.reserve(sync_frames.size());
  for (const auto &f : sync_frames) {
    if (unique_frames.empty() || unique_frames.back().time_stem != f.time_stem) {
      unique_frames.push_back(f);
    } else {
      LOG_WARN("Run: Dropping duplicate sync frame with time_stem: " << f.time_stem);
    }
  }
  sync_frames = std::move(unique_frames);

  int processed_frames = 0;

#pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < sync_frames.size(); ++i) {
    const auto &frame = sync_frames[i];
    processFrame(frame.curr_gnss, frame.next_gnss, frame.matched_files, frame.time_stem);
    
    int current_processed;
#pragma omp atomic capture
    {
      processed_frames++;
      current_processed = processed_frames;
    }
    
    LOG_INFO("Run: Finish merge frame " << current_processed << " / " << sync_frames.size()
             << " (GNSS timestamp: " << std::setprecision(13) << frame.curr_gnss.timestamp << ")");
  }

  LOG_INFO("Run: LiDAR_Dynamic_Merge finished running. Processed "
           << processed_frames << " synchronized frames.");

  writeEgoTrajectory(sync_frames);
}

} // namespace lidar_dynamic_merge