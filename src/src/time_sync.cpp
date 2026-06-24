#include "lidar_dynamicmerge_c/time_sync.hpp"
#include <algorithm>
#include <cmath>

namespace lidar_dynamic_merge {

std::vector<SyncFrame> synchronizeFrames(
    const std::vector<GNSSData> &gnss_data_list,
    const std::map<int, std::vector<lidar_utils::LidarContent>> &lidar_files,
    double gnss_freq, double gnss_std_thres) {

  std::vector<SyncFrame> sync_frames;

  // Synchronization threshold (e.g., 0.05 seconds = 50 ms)
  double sync_threshold = 1.0 / gnss_freq / 2;

  // Enforce GNSS frequency downsampling (e.g. 10Hz data -> 1Hz processing)
  double process_interval = 1.0 / gnss_freq * 0.6;
  double last_processed_time = -1.0;

  // Track the last matched index to optimize searching sequentially
  std::map<int, size_t> search_start_idx;
  for (const auto &[id, file_list] : lidar_files) {
    search_start_idx[id] = 0;
  }

  GNSSData curr_gnss;
  std::map<int, std::string> curr_matched_files;
  bool has_curr = false;

  for (const auto &gnss : gnss_data_list) {
    // 1. Reject bad GNSS data based on threshold
    if (gnss.position_std.norm() > gnss_std_thres) {
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
    for (const auto &[id, file_list] : lidar_files) {
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
        SyncFrame frame;
        frame.curr_gnss = curr_gnss;
        frame.next_gnss = gnss;
        frame.matched_files = curr_matched_files;
        sync_frames.push_back(frame);
      }
      curr_gnss = gnss;
      curr_matched_files = matched_files;
      has_curr = true;
    }
  }

  return sync_frames;
}

} // namespace lidar_dynamic_merge
