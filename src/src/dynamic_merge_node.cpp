#include "lidar_dynamicmerge_c/dynamic_merge_node.hpp"
#include <algorithm>
#include <cmath>
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
    const std::map<int, std::string> &matched_files) {
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
    std::string out_file = out_source_ + "/" +
                           std::to_string(static_cast<uint64_t>(
                               std::round(gnss.timestamp * 1e3))) +
                           ".pcd";
    lidar_utils::CloudUtils::saveFile(merged_cloud, out_file, out_format_);
  }
}

void DynamicMergeNode::runBagPipelined(const std::vector<SyncFrame> &sync_frames) {
  const size_t total_frames = sync_frames.size();
  BoundedQueue<FramePayload> queue(32);
  std::atomic<int> processed_frames{0};

  // Launch sequential producer thread (reads rosbag sequentially from start to finish)
  std::thread producer([&]() {
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
      LOG_INFO("Run: Finish merge GNSS timestamp: "
               << std::setprecision(13) << payload.next_gnss.timestamp
               << ", frame: " << current_processed << " / " << total_frames);
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
    std::string out_file = out_source_ + "/" +
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
  if (sync_frames.empty()) {
    LOG_FATAL("Time Sync: Synchronization resulted in 0 frames! Please check GNSS frequency, std threshold, and LiDAR timestamps overlap.");
    return;
  }

  if (bag_loader_) {
    runBagPipelined(sync_frames);
    return;
  }

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