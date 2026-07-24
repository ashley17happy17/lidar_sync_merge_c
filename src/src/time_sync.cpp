#include "lidar_dynamicmerge_c/time_sync.hpp"
#include "lidar_dynamicmerge_c/logger.hpp"
#include <algorithm>
#include <cmath>

namespace lidar_dynamic_merge {

std::vector<SyncFrame> synchronizeFrames(
    const std::vector<GNSSData> &gnss_data_list,
    const std::map<int, std::vector<lidar_utils::LidarContent>> &lidar_files,
    double gnss_freq, double gnss_std_thres, double lidar_hz) {

  std::vector<SyncFrame> sync_frames;

  // Max allowed gap between the GNSS timestamp and the LiDAR frame it syncs
  // to (e.g., 10 Hz LiDAR -> 0.1s)
  double sync_threshold = 1.0 / lidar_hz;

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
      LOG_WARN("Time Sync: GNSS rejected due to high standard deviation: "
               << gnss.position_std.norm() << " > " << gnss_std_thres);
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

      // Binary search starting from the last matched index. lower_bound
      // returns the first LiDAR frame whose timestamp is >= the GNSS
      // timestamp, i.e. the earliest LiDAR frame at or after the GNSS fix.
      auto it = std::lower_bound(file_list.begin() + start_idx, file_list.end(),
                                 target_timestamp,
                                 [](const lidar_utils::LidarContent &a,
                                    double val) { return a.timestamp < val; });

      // No LiDAR frame at or after this GNSS timestamp is available.
      if (it == file_list.end()) {
        LOG_WARN("Time Sync: LiDAR "
                 << id << " failed sync, no LiDAR frame at or after GNSS time "
                 << std::setprecision(13) << target_timestamp);
        all_matched = false;
        break;
      }

      double diff = it->timestamp - target_timestamp;

      if (diff > sync_threshold) {
        LOG_WARN("Time Sync: LiDAR "
                 << id << " failed sync, diff (" << diff
                 << "s) > threshold (" << sync_threshold << "s) at GNSS time "
                 << std::setprecision(13) << target_timestamp);
        all_matched = false;
        break;
      }

      // Cache index to start from here next time
      search_start_idx[id] = std::distance(file_list.begin(), it);
      matched_files[id] = it->filename;
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

  if (sync_frames.empty()) {
    LOG_FATAL(
        "Time Sync: Synchronization resulted in 0 frames! Please check GNSS "
        "frequency, std threshold, and LiDAR timestamps overlap.");
  } else {
    LOG_INFO("Time Sync: Successfully synchronized " << sync_frames.size()
                                                     << " frames.");
  }

  return sync_frames;
}

} // namespace lidar_dynamic_merge
