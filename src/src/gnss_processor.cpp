#include "lidar_dynamicmerge_c/gnss_processor.hpp"
#include "lidar_dynamicmerge_c/logger.hpp"
#include <cmath>
#include <iomanip>

namespace lidar_dynamic_merge {

void WGS84toTWD97(double lat, double lon, double &x, double &y) {
  double a = 6378137.0;
  double b = 6356752.3141403;
  double lon0 = 121 * M_PI / 180.0;
  double k0 = 0.9999;
  double dx = 250000;

  double lat_rad = lat * M_PI / 180.0;
  double lon_rad = lon * M_PI / 180.0;

  double e2 = 1 - std::pow(b / a, 2);
  double e_p2 = e2 / (1 - e2);

  double N = a / std::sqrt(1 - e2 * std::pow(std::sin(lat_rad), 2));
  double T = std::pow(std::tan(lat_rad), 2);
  double C = e_p2 * std::pow(std::cos(lat_rad), 2);
  double A = std::cos(lat_rad) * (lon_rad - lon0);

  double M =
      a *
      ((1 - e2 / 4 - 3 * std::pow(e2, 2) / 64 - 5 * std::pow(e2, 3) / 256) *
           lat_rad -
       (3 * e2 / 8 + 3 * std::pow(e2, 2) / 32 + 45 * std::pow(e2, 3) / 1024) *
           std::sin(2 * lat_rad) +
       (15 * std::pow(e2, 2) / 256 + 45 * std::pow(e2, 3) / 1024) *
           std::sin(4 * lat_rad) -
       (35 * std::pow(e2, 3) / 3072) * std::sin(6 * lat_rad));

  x = dx + k0 * N *
               (A + (1 - T + C) * std::pow(A, 3) / 6 +
                (5 - 18 * T + std::pow(T, 2) + 72 * C - 58 * e_p2) *
                    std::pow(A, 5) / 120);

  y = k0 *
      (M + N * std::tan(lat_rad) *
               (std::pow(A, 2) / 2 +
                (5 - T + 9 * C + 4 * std::pow(C, 2)) * std::pow(A, 4) / 24 +
                (61 - 58 * T + std::pow(T, 2) + 600 * C - 330 * e_p2) *
                    std::pow(A, 6) / 720));
}

void processGNSSData(const std::string &gnss_file_path,
                     std::vector<GNSSData> &gnss_data_list,
                     Eigen::Vector3d &local_origin) {
  LOG_INFO("Processing GNSS data with WGS84 -> TWD97-2010 projection -> Local "
           "Cartesian: "
           << gnss_file_path);
  std::ifstream gnss_file(gnss_file_path);

  if (!gnss_file.is_open()) {
    LOG_ERROR("Could not open the file.");
    exit(1);
  }

  std::string line, word;
  bool origin_set = false;

  while (std::getline(gnss_file, line)) {
    std::vector<std::string> row;
    std::stringstream ss(line);

    // Split individual line by comma
    while (std::getline(ss, word, ',')) {
      row.push_back(word);
    }

    if (row.size() < 11)
      continue;

    try {
      // Extract raw Lat/Lon/H
      double lat = std::stod(row[1]);
      double lon = std::stod(row[2]);
      double h = std::stod(row[3]);

      // Convert WGS84 to TWD97 (EPSG:3826)
      double x = 0.0, y = 0.0;
      WGS84toTWD97(lat, lon, x, y);
      double z =
          h; // Or h - geoidSep if orthometric height is strictly required

      // Capture the first valid global TWD97 point as the Local Origin
      if (!origin_set) {
        local_origin = Eigen::Vector3d(x, y, z);
        origin_set = true;
        LOG_INFO(std::fixed << std::setprecision(10)
                            << "Set Local Cartesian Origin to TWD97: X=" << x
                            << ", Y=" << y << ", Z=" << z);
      }

      // Transform to Local Cartesian
      x -= local_origin.x();
      y -= local_origin.y();
      z -= local_origin.z();

      // Keep RPY in degrees as expected by motion_impl
      double roll = std::stod(row[7]);
      double pitch = std::stod(row[8]);

      // Heading: input is North=0, East=90 (Clockwise).
      // Standard ENU Yaw is East=0, North=90 (Counter-Clockwise).
      double heading_deg = std::stod(row[9]);
      double yaw = 90.0 - heading_deg;

      // Normalize yaw to [-180, 180]
      while (yaw > 180.0)
        yaw -= 360.0;
      while (yaw < -180.0)
        yaw += 360.0;

      GNSSData data;
      data.timestamp = std::stod(row[0]);
      data.pose_6d = Eigen::VectorXd(6);
      data.pose_6d << x, y, z, roll, pitch, yaw;
      data.position_std = Eigen::Vector3d(std::stod(row[4]), std::stod(row[5]),
                                          std::stod(row[6]));
      data.geoidSep = std::stod(row[10]);
      gnss_data_list.push_back(data);
    } catch (...) {
      // Ignore header or malformed rows
    }
  }
  gnss_file.close();
  LOG_INFO("Successfully loaded " << gnss_data_list.size() << " GNSS records.");
}

} // namespace lidar_dynamic_merge
