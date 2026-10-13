#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include "lidar_dynamicmerge_c/types.hpp"

namespace lidar_dynamic_merge {

// Geodetic projection: WGS84 (lat/lon in degrees) to Taiwan TWD97 TM2 projection (EPSG:3826)
void WGS84toTWD97(double lat, double lon, double &x, double &y);

// Normalizes clockwise compass heading (North=0, East=90) to ENU yaw (East=0, North=90, CCW [-180, 180])
double headingToEnuYaw(double heading_deg);

// Converts a single raw GNSS fix (from CSV or ROS topic) to local Cartesian GNSSData
bool convertRawFixToGNSSData(double timestamp, double lat, double lon, double alt,
                             double roll, double pitch, double heading,
                             const Eigen::Vector3d &std_xyz, double geoid_sep,
                             Eigen::Vector3d &local_origin, bool &origin_set,
                             GNSSData &out_data);

// Reads and parses GNSS CSV file (backwards compatibility / FileLoader)
void processGNSSData(const std::string &gnss_file_path,
                     std::vector<GNSSData> &gnss_data_list,
                     Eigen::Vector3d &local_origin);

} // namespace lidar_dynamic_merge
