#pragma once

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <lidar_utils/types.hpp>
#include "lidar_dynamicmerge_c/types.hpp"

namespace lidar_dynamic_merge {

class BagLoader {
public:
  explicit BagLoader(const std::string &bag_file_path);
  ~BagLoader();

  bool open();
  void close();

  // Load GNSS/Pose from topic (supports nav_msgs/Odometry or sensor_msgs/NavSatFix)
  bool loadGnssTopic(const std::string &topic,
                     std::vector<GNSSData> &gnss_list,
                     Eigen::Vector3d &local_origin,
                     const std::string &imu_topic = "");

  // Load IMU angular velocity and acceleration from sensor_msgs/Imu
  bool loadImuTopic(const std::string &topic,
                    std::vector<lidar_utils::ImuSample> &imu_list);

  // Load Odometry velocity from nav_msgs/Odometry or geometry_msgs/TwistStamped
  bool loadOdomTopic(const std::string &topic, double odom_scale,
                     std::vector<lidar_utils::OdomSample> &odom_list);

  // Index LiDAR topic: reads message headers (stamp), populates file_list with virtual indices
  bool indexLidarTopic(int lidar_id, const std::string &topic, double time_offset,
                       std::vector<lidar_utils::LidarContent> &file_list);

  // Unified single-pass telemetry loader (GNSS, IMU, ODOM)
  bool loadTelemetry(const std::string &gnss_topic,
                     const std::string &imu_topic,
                     const std::string &odom_topic,
                     double odom_scale,
                     std::vector<GNSSData> &gnss_list,
                     Eigen::Vector3d &local_origin,
                     std::vector<lidar_utils::ImuSample> &imu_list,
                     std::vector<lidar_utils::OdomSample> &odom_list);

  // Unified single-pass indexer for all LiDAR topics
  bool indexAllLidarTopics(const std::map<int, LidarConfig> &lidars,
                           std::map<int, std::vector<lidar_utils::LidarContent>> &lidar_files);

  // Fast reader returning raw PointCloud2 message for pipelined processing
  sensor_msgs::PointCloud2::ConstPtr readPointCloud2Msg(int lidar_id, size_t msg_index);

  // Thread-safe point cloud reader for processFrame()
  bool readPointCloud(int lidar_id, size_t msg_index,
                      lidar_utils::SensorType sensor_type,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                      std::vector<double> &timestamps);

  // Fast branch-free point cloud unpacker
  static bool unpackPointCloud2(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                lidar_utils::SensorType sensor_type,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                std::vector<double> &timestamps,
                                double time_offset = 0.0);

private:

  std::string bag_file_path_;
  rosbag::Bag bag_;
  bool is_open_ = false;
  std::mutex bag_mutex_;

  // Stores lightweight MessageInstances per LiDAR for on-demand lazy reading
  std::map<int, std::vector<rosbag::MessageInstance>> lidar_messages_;
};

} // namespace lidar_dynamic_merge

