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
    std::cout << "[INFO] Loading file list for lidar " << id << ": "
              << config.path << std::endl;

    std::vector<lidar_utils::LidarContent> file_list;
    lidar_utils::CloudUtils::readContent(config.path, file_list);

    lidar_files_[id] = file_list;
    std::cout << "[INFO] Lidar " << id << " successfully loaded "
              << file_list.size() << " frames." << std::endl;
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
    // === Transform from Global Frame to GNSS Frame ===
    Eigen::Matrix4d predicted_pose_inv = gnss.getTransform().inverse();
    lidar_utils::CloudUtils::directGeoreference(merged_cloud, merged_cloud,
                                                predicted_pose_inv);

    // === Transform from GNSS Frame to out_lidar Frame ===
    Eigen::Matrix4d inv_ext;
    inv_ext = lidar_utils::CloudUtils::getExtrinsics(lidars_[out_lidar_].type,
                                                     lidars_[out_lidar_].trans,
                                                     lidars_[out_lidar_].rot)
                  .inverse();
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
  std::cout << "[INFO] Starting synchronization loop...\n"
            << "---------------------------------------------------------\n";

  auto sync_frames = synchronizeFrames(gnss_data_list_, lidar_files_,
                                       gnss_freq_, gnss_std_thres_);
  int processed_frames = 0;

  for (const auto &frame : sync_frames) {
    processFrame(frame.curr_gnss, &frame.next_gnss, frame.matched_files);
    processed_frames++;
    std::cout << "[INFO] Finish merge GNSS timestamp:"
              << std::setprecision(13) << frame.next_gnss.timestamp
              << ", frame: " << processed_frames << std::endl;
  }

  std::cout << "[INFO] LiDAR_Dynamic_Merge finished running. Processed "
            << processed_frames << " synchronized frames.\n";
}

} // namespace lidar_dynamic_merge