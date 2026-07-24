#include "lidar_dynamicmerge_c/dynamic_merge_node.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace lidar_dynamic_merge {

DynamicMergeNode::DynamicMergeNode(const std::string &config_file) {
  loadConfig(config_file);
  loadGnssData();
  loadLidarContent();
}

void DynamicMergeNode::loadGnssData() {
  Eigen::Vector3d local_origin;
  processGNSSData(gnss_file_, gnss_data_list_, local_origin);
}

void DynamicMergeNode::loadLidarContent() {
  for (const auto &[id, config] : lidars_) {
    LOG_INFO("Load LiDAR: Loading file list for LiDAR " << id << ": "
                                                        << config.path);

    std::vector<lidar_utils::LidarContent> file_list;
    lidar_utils::CloudUtils::readContent(config.path, file_list);

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

void DynamicMergeNode::processFrame(
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
    processFrame(frame.curr_gnss, &frame.next_gnss, frame.matched_files);
    
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