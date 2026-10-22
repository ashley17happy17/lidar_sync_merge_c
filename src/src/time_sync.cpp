#include "lidar_dynamicmerge_c/time_sync.hpp"
#include "lidar_dynamicmerge_c/logger.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>

namespace lidar_dynamic_merge {

std::vector<SyncFrame> synchronizeFrames(
    const std::vector<GNSSData> &gnss_data_list,
    const std::map<int, std::vector<lidar_utils::LidarContent>> &lidar_files,
    double gnss_freq, double gnss_std_thres, double lidar_hz) {

  std::vector<SyncFrame> sync_frames;

  if (lidar_files.empty() || gnss_data_list.empty()) {
    LOG_FATAL("Time Sync: Empty LiDAR or GNSS data provided to synchronizeFrames.");
    return sync_frames;
  }

  // 1. Identify reference LiDAR: prefer LiDAR 1, or first available
  int ref_id = lidar_files.count(1) ? 1 : lidar_files.begin()->first;
  const auto &ref_frames = lidar_files.at(ref_id);
  if (ref_frames.empty()) {
    LOG_FATAL("Time Sync: Reference LiDAR " << ref_id << " has 0 frames!");
    return sync_frames;
  }

  // Max allowed gap between LiDAR frames for multi-LiDAR sync
  double sync_threshold = 1.0 / lidar_hz;

  // Enforce frequency downsampling if requested (e.g. 10Hz data -> 1Hz processing)
  double process_interval = (gnss_freq > 0.0) ? (1.0 / gnss_freq * 0.6) : 0.0;
  double last_processed_time = -1.0;

  // Track search start indices for other LiDARs
  std::map<int, size_t> search_start_idx;
  for (const auto &[id, file_list] : lidar_files) {
    search_start_idx[id] = 0;
  }

  const double gnss_start_time = gnss_data_list.front().timestamp;
  const double gnss_end_time = gnss_data_list.back().timestamp;

  for (size_t k = 0; k < ref_frames.size(); ++k) {
    const double t_ref = ref_frames[k].timestamp;

    // Reject LiDAR frames outside the GNSS recording span
    if (t_ref < gnss_start_time - sync_threshold || t_ref > gnss_end_time + sync_threshold) {
      LOG_WARN("Time Sync: LiDAR frame at " << std::setprecision(13) << t_ref
               << " is outside GNSS time span [" << gnss_start_time << ", " << gnss_end_time << "], skipping.");
      continue;
    }

    // Downsampling check
    if (last_processed_time > 0.0 && (t_ref - last_processed_time) < process_interval) {
      continue;
    }

    // 2. Synchronize all other LiDARs to reference LiDAR
    bool all_lidars_matched = true;
    std::map<int, std::string> matched_files;
    matched_files[ref_id] = ref_frames[k].filename;

    for (const auto &[id, file_list] : lidar_files) {
      if (id == ref_id) continue;
      if (file_list.empty()) {
        all_lidars_matched = false;
        break;
      }

      size_t start_idx = search_start_idx[id];
      auto it = std::lower_bound(file_list.begin() + start_idx, file_list.end(),
                                 t_ref,
                                 [](const lidar_utils::LidarContent &a, double val) {
                                   return a.timestamp < val;
                                 });

      double min_diff = 1e9;
      std::string matched_file;
      size_t matched_idx = start_idx;

      if (it == file_list.end()) {
        auto prev = it - 1;
        min_diff = std::abs(prev->timestamp - t_ref);
        matched_file = prev->filename;
        matched_idx = std::distance(file_list.begin(), prev);
      } else if (it == file_list.begin()) {
        min_diff = std::abs(it->timestamp - t_ref);
        matched_file = it->filename;
        matched_idx = std::distance(file_list.begin(), it);
      } else {
        auto prev = it - 1;
        double diff_prev = std::abs(prev->timestamp - t_ref);
        double diff_it = std::abs(it->timestamp - t_ref);
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
        LOG_WARN("Time Sync: LiDAR " << id << " failed sync to ref LiDAR at t="
                 << std::setprecision(13) << t_ref << ", min_diff (" << min_diff
                 << "s) > threshold (" << sync_threshold << "s)");
        all_lidars_matched = false;
        break;
      }

      search_start_idx[id] = matched_idx;
      matched_files[id] = matched_file;
    }

    if (!all_lidars_matched) {
      continue;
    }

    // 3. Interpolate GNSS data at t_ref
    GNSSData curr_gnss = interpolateGnss(t_ref, gnss_data_list);
    if (curr_gnss.position_std.norm() > gnss_std_thres) {
      LOG_WARN("Time Sync: LiDAR frame at " << std::setprecision(13) << t_ref
               << " rejected due to high GNSS std: " << curr_gnss.position_std.norm()
               << " > " << gnss_std_thres);
      continue;
    }

    // Interpolate next GNSS data at next frame epoch for motion compensation
    double t_next = (k + 1 < ref_frames.size()) ? ref_frames[k + 1].timestamp : (t_ref + 1.0 / lidar_hz);
    GNSSData next_gnss = interpolateGnss(t_next, gnss_data_list);

    SyncFrame frame;
    frame.curr_gnss = curr_gnss;
    frame.next_gnss = next_gnss;
    frame.matched_files = matched_files;
    frame.lidar_time = t_ref;

    // Time stem
    std::string stem = std::filesystem::path(ref_frames[k].filename).stem().string();
    if (!stem.empty() && stem.find_first_not_of("0123456789") == std::string::npos && stem.size() >= 10) {
      frame.time_stem = stem;
    } else {
      frame.time_stem = std::to_string(static_cast<uint64_t>(std::round(t_ref * 1e3)));
    }

    sync_frames.push_back(frame);
    last_processed_time = t_ref;
  }

  if (sync_frames.empty()) {
    LOG_FATAL(
        "Time Sync: Synchronization resulted in 0 frames! Please check GNSS "
        "frequency, std threshold, and LiDAR timestamps overlap.");
  } else {
    LOG_INFO("Time Sync: Successfully synchronized " << sync_frames.size()
                                                     << " frames (LiDAR-driven).");
  }

  return sync_frames;
}

} // namespace lidar_dynamic_merge
