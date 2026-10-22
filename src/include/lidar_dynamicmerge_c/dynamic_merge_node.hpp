#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <lidar_utils/cloud_utils.hpp>
#include "lidar_dynamicmerge_c/bag_loader.hpp"
#include "lidar_dynamicmerge_c/file_loader.hpp"
#include "lidar_dynamicmerge_c/gnss_processor.hpp"
#include "lidar_dynamicmerge_c/logger.hpp"
#include "lidar_dynamicmerge_c/time_sync.hpp"
#include "lidar_dynamicmerge_c/types.hpp"
#include <map>
#include <memory>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace lidar_dynamic_merge {

template <typename T>
class BoundedQueue {
public:
  explicit BoundedQueue(size_t max_size) : max_size_(max_size), finished_(false) {}

  void push(T item) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_not_full_.wait(lock, [this]() { return queue_.size() < max_size_ || finished_; });
    if (finished_) return;
    queue_.push(std::move(item));
    cv_not_empty_.notify_one();
  }

  bool pop(T &item) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_not_empty_.wait(lock, [this]() { return !queue_.empty() || finished_; });
    if (queue_.empty()) return false;
    item = std::move(queue_.front());
    queue_.pop();
    cv_not_full_.notify_one();
    return true;
  }

  void finish() {
    std::unique_lock<std::mutex> lock(mutex_);
    finished_ = true;
    cv_not_empty_.notify_all();
    cv_not_full_.notify_all();
  }

private:
  size_t max_size_;
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable cv_not_empty_;
  std::condition_variable cv_not_full_;
  bool finished_;
};

struct FramePayload {
  size_t frame_index = 0;
  GNSSData curr_gnss;
  GNSSData next_gnss;
  std::map<int, sensor_msgs::PointCloud2::ConstPtr> lidar_msgs;
  std::string time_stem;
  double lidar_time = 0.0;
};

class DynamicMergeNode {
public:
  DynamicMergeNode(const std::string &config_file);
  ~DynamicMergeNode() = default;

  void run();

private:
  void loadConfig(const std::string &config_file);
  void loadData();
  void buildGnssSamples();
  void processFrame(const GNSSData &gnss, const GNSSData &next_gnss,
                    const std::map<int, std::string> &matched_files,
                    const std::string &time_stem = "");
  void runBagPipelined(std::vector<SyncFrame> &sync_frames);
  void processFrameFromPayload(const FramePayload &payload);
  std::string computeTimeStem(const GNSSData &gnss,
                              const std::map<int, std::string> &matched_files,
                              double *out_time_sec = nullptr) const;
  GNSSData interpolateGnss(double t, const std::vector<GNSSData> &gnss_list) const;
  void interpolateImuOrientation(double t, const std::vector<ImuOrientation> &imu_list,
                                 double &roll_rad, double &pitch_rad, double &yaw_rad) const;
  void writeEgoTrajectory(const std::vector<SyncFrame> &sync_frames);
  // Vehicle pose in the local world frame at an arbitrary epoch, obtained by
  // interpolating the GNSS trajectory (lerp on position, slerp on attitude).
  // Clamps to the first/last record outside the trajectory.
  Eigen::Matrix4d interpolatePoseAt(double t) const;

  YAML::Node config_;
  std::vector<GNSSData> gnss_data_list_;
  // Lightweight position-only view of the GNSS trajectory, used as the
  // translation source for motion compensation.
  std::vector<lidar_utils::GnssSample> gnss_samples_;
  // IMU gyro (+ optional accel) samples in the GNSS/vehicle FLU frame: the
  // rotation source always, and the translation source for IMU_ACC.
  std::vector<lidar_utils::ImuSample> imu_data_list_;
  // Odometer velocity samples (vehicle FLU frame): translation source for ODOM.
  std::vector<lidar_utils::OdomSample> odom_data_list_;

  // Config parameters
  std::string bag_file_;
  std::unique_ptr<BagLoader> bag_loader_;

  std::string gnss_source_;
  std::string imu_source_;
  std::string odom_source_;
  // Factor applied to the odom velocity columns to reach m/s (1.0 for a
  // file already in m/s, 1/3.6 for a CAN speed logged in km/h).
  double odom_scale_ = 1.0;
  // Translation source for motion compensation (rotation is always the gyro).
  lidar_utils::MotionMethod motion_method_ =
      lidar_utils::MotionMethod::GNSS_TRANS;
  // Use the legacy deskew instead: per-point GNSS pose interpolated between the
  // current and next fix, no IMU. Kept for comparison against the newer methods;
  // motion_method_ is unused when this is set.
  bool use_legacy_motion_ = false;
  double gnss_std_thres_;
  double gnss_freq_;
  double lidar_hz_;

  std::map<int, LidarConfig> lidars_;
  std::map<int, std::vector<lidar_utils::LidarContent>>
      lidar_files_; // Lidar ID -> files sorted by timestamp

  std::vector<double> crop_min_;
  std::vector<double> crop_max_;
  double denoise_radius_;
  double denoise_epsilon_;
  double ds_voxel_size_;
  bool motion_enable_;
  double merge_voxel_size_;
  int out_lidar_;
  std::string out_source_;
  std::string out_traj_;
  lidar_utils::FileFormat out_format_;
  Eigen::Vector3d out_la_;
  Eigen::Vector3d out_bs_;
};

} // namespace lidar_dynamic_merge