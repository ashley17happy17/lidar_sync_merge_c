#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include "lidar_dynamicmerge_c/types.hpp"

namespace lidar_dynamic_merge {

void processGNSSData(const std::string& gnss_file_path, std::vector<GNSSData>& gnss_data_list, Eigen::Vector3d& local_origin);

} // namespace lidar_dynamic_merge
