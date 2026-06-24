#pragma once

#include "lidar_dynamicmerge_c/gnss_processor.hpp"
#include <lidar_utils/types.hpp>
#include <map>
#include <string>
#include <vector>

#include "lidar_dynamicmerge_c/types.hpp"

namespace lidar_dynamic_merge {

std::vector<SyncFrame> synchronizeFrames(
    const std::vector<GNSSData> &gnss_data_list,
    const std::map<int, std::vector<lidar_utils::LidarContent>> &lidar_files,
    double gnss_freq, double gnss_std_thres);

} // namespace lidar_dynamic_merge
