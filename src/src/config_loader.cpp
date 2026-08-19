#include "lidar_dynamicmerge_c/dynamic_merge_node.hpp"
#include <filesystem>
#include <iostream>
namespace lidar_dynamic_merge {

void DynamicMergeNode::loadConfig(const std::string &config_file) {
  config_ = YAML::LoadFile(config_file);

  gnss_file_ = config_["gnss_file"].as<std::string>();
  // Optional: IMU csv for gyro-based rotation compensation. If absent, motion
  // compensation runs without rotation deskew.
  if (config_["imu_file"])
    imu_file_ = config_["imu_file"].as<std::string>();
  // Optional: odometer csv (timestamp, vx[, vy, vz]) for ODOM_TRANS.
  if (config_["odom_file"])
    odom_file_ = config_["odom_file"].as<std::string>();
  // Unit of the odom velocity columns. The library works in m/s, but a CAN
  // vehicle-speed log is typically km/h, and feeding that in raw scales every
  // deskew translation by 3.6.
  if (config_["odom_speed_unit"]) {
    std::string u = config_["odom_speed_unit"].as<std::string>();
    if (u == "mps" || u == "m/s")
      odom_scale_ = 1.0;
    else if (u == "kph" || u == "km/h")
      odom_scale_ = 1.0 / 3.6;
    else {
      LOG_WARN("Load Config: Unknown odom_speed_unit '"
               << u << "', assuming m/s.");
    }
  }
  // Escape hatch for anything the named units do not cover (e.g. raw counts):
  // takes precedence over odom_speed_unit.
  if (config_["odom_scale"])
    odom_scale_ = config_["odom_scale"].as<double>();
  // Motion-compensation translation source: GNSS (default), ODOM, or IMU_ACC.
  if (config_["motion_method"]) {
    std::string m = config_["motion_method"].as<std::string>();
    if (m == "GNSS")
      motion_method_ = lidar_utils::MotionMethod::GNSS_TRANS;
    else if (m == "ODOM")
      motion_method_ = lidar_utils::MotionMethod::ODOM_TRANS;
    else if (m == "IMU_ACC")
      motion_method_ = lidar_utils::MotionMethod::IMU_ACC_TRANS;
    else if (m == "LEGACY")
      use_legacy_motion_ = true;
    else
      LOG_WARN("Load Config: Unknown motion_method '"
               << m << "', defaulting to GNSS.");
  }
  gnss_std_thres_ = config_["gnss_std_thres"].as<double>();
  gnss_freq_ = config_["gnss_freq"].as<double>();
  lidar_hz_ = config_["lidar_hz"].as<double>();
  // Clock offset between the LiDAR timestamps and the GNSS clock, in seconds,
  // ADDED to every LiDAR time. A LiDAR running 1.145 s fast needs -1.145.
  // Applies to all LiDARs; lidarN_time_offset overrides it for one sensor.
  double lidar_time_offset = 0.0;
  if (config_["lidar_time_offset"])
    lidar_time_offset = config_["lidar_time_offset"].as<double>();

  // Load lidar configs (1 to 5)
  for (int i = 1; i <= 5; ++i) {
    std::string prefix = "lidar" + std::to_string(i) + "_";
    if (config_[prefix + "fp"]) {
      std::string path = config_[prefix + "fp"].as<std::string>();
      if (path.empty()) {
        LOG_WARN("Load Config: Skipping LiDAR " << i
                                                << " because fp is empty.");
        continue;
      }

      LidarConfig lc;
      lc.path = path;
      if (config_[prefix + "type"]) {
        std::string type_str = config_[prefix + "type"].as<std::string>();
        if (type_str == "OUSTER_OS1_128")
          lc.type = lidar_utils::SensorType::OUSTER_OS1_128;
        else if (type_str == "OUSTER_OS1_32")
          lc.type = lidar_utils::SensorType::OUSTER_OS1_32;
        else if (type_str == "VELODYNE_VLP16")
          lc.type = lidar_utils::SensorType::VELODYNE_VLP16;
        else if (type_str == "VELODYNE_VLS128")
          lc.type = lidar_utils::SensorType::VELODYNE_VLS128;
        else {
          LOG_FATAL("Load Config: Wrong LiDAR Type input.");
          exit(1);
        }
      } else {
        LOG_FATAL("Load Config: Wrong LiDAR Type input.");
        exit(1);
      }
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

      lc.time_offset = config_[prefix + "time_offset"]
                           ? config_[prefix + "time_offset"].as<double>()
                           : lidar_time_offset;
      if (lc.time_offset != 0.0)
        LOG_INFO("Load Config: LiDAR " << i << " timestamps shifted by "
                                       << lc.time_offset * 1e3
                                       << " ms to match the GNSS clock.");

      lidars_[i] = lc;
    }
  }

  if (lidars_.empty()) {
    LOG_FATAL("Load Config: All lidar file paths are empty. Exiting.");
    exit(1);
  }

  crop_min_ = config_["crop_min_bound"].as<std::vector<double>>();
  crop_max_ = config_["crop_max_bound"].as<std::vector<double>>();
  denoise_radius_ = config_["denoise_radius"].as<double>();
  denoise_epsilon_ = config_["denoise_epsilon"].as<double>();
  ds_voxel_size_ = config_["ds_voxel_size"].as<double>();
  motion_enable_ = config_["motion_enable"].as<bool>();
  merge_voxel_size_ = config_["merge_voxel_size"].as<double>();

  // State the active deskew configuration explicitly: without this a wrong
  // motion_method or a missing source file leaves no trace in the log.
  if (use_legacy_motion_) {
    LOG_INFO("Load Config: Motion compensation "
             << (motion_enable_ ? "ENABLED" : "DISABLED")
             << ", method = LEGACY (per-point GNSS pose interpolation between "
                "the current and next fix; no IMU, and the scan tail beyond the "
                "next fix is clamped)");
  } else {
    LOG_INFO("Load Config: Motion compensation "
             << (motion_enable_ ? "ENABLED" : "DISABLED")
             << ", rotation = IMU gyro, translation = "
             << (motion_method_ == lidar_utils::MotionMethod::ODOM_TRANS
                     ? "ODOM"
                     : motion_method_ == lidar_utils::MotionMethod::IMU_ACC_TRANS
                           ? "IMU_ACC"
                           : "GNSS"));
  }

  out_lidar_ = config_["out_lidar"].as<int>();
  out_fp_ = config_["out_fp"].as<std::string>();
  if (!out_fp_.empty()) {
    std::filesystem::create_directories(out_fp_);
  }
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
  if (out_la_.isZero() && out_bs_.isZero()) {
    LOG_INFO("Load Config: out_la and out_bs are zero, output to LiDAR "
             << out_lidar_ << " center.");
  } else {
    LOG_INFO("Load Config: output to position with leverarm(m) ["
             << out_la_[0] << ", " << out_la_[1] << ", " << out_la_[2]
             << "] and boresight (deg) [" << out_bs_[0] << ", " << out_bs_[1]
             << ", " << out_bs_[2] << "].");
  }
}

} // namespace lidar_dynamic_merge
