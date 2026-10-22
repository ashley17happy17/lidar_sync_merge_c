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

double headingToEnuYaw(double heading_deg) {
  // Heading: input is North=0, East=90 (Clockwise).
  // Standard ENU Yaw is East=0, North=90 (Counter-Clockwise).
  double yaw = 90.0 - heading_deg;
  while (yaw > 180.0)
    yaw -= 360.0;
  while (yaw < -180.0)
    yaw += 360.0;
  return yaw;
}

bool convertRawFixToGNSSData(double timestamp, double lat, double lon, double alt,
                             double roll, double pitch, double heading,
                             const Eigen::Vector3d &std_xyz, double geoid_sep,
                             Eigen::Vector3d &local_origin, bool &origin_set,
                             GNSSData &out_data) {
  double x = 0.0, y = 0.0;
  WGS84toTWD97(lat, lon, x, y);
  double z = alt;

  out_data.timestamp = timestamp;
  out_data.lat = lat;
  out_data.lon = lon;
  out_data.alt = alt;
  out_data.twd97_x = x;
  out_data.twd97_y = y;
  out_data.twd97_z = z;
  out_data.raw_roll = roll;
  out_data.raw_pitch = pitch;
  double norm_yaw = heading;
  while (norm_yaw > 180.0) norm_yaw -= 360.0;
  while (norm_yaw < -180.0) norm_yaw += 360.0;
  out_data.raw_yaw = norm_yaw;

  if (!origin_set) {
    local_origin = Eigen::Vector3d(x, y, z);
    origin_set = true;
    LOG_INFO(
        std::fixed
        << std::setprecision(10)
        << "Process GNSS: Projecting GNSS from WGS84 -> TWD97-2010 "
           "projection -> Local Cartesian, set Local Cartesian Origin to "
           "TWD97: X="
        << x << ", Y=" << y << ", Z=" << z);
  }

  x -= local_origin.x();
  y -= local_origin.y();
  z -= local_origin.z();

  double yaw = headingToEnuYaw(heading);

  out_data.timestamp = timestamp;
  out_data.pose_6d = Eigen::VectorXd(6);
  out_data.pose_6d << x, y, z, roll, pitch, yaw;
  out_data.position_std = std_xyz;
  out_data.geoidSep = geoid_sep;
  return true;
}

void processGNSSData(const std::string &gnss_file_path,
                     std::vector<GNSSData> &gnss_data_list,
                     Eigen::Vector3d &local_origin) {
  LOG_INFO("Process GNSS: Loading GNSS File: " << gnss_file_path);
  std::ifstream gnss_file(gnss_file_path);

  if (!gnss_file.is_open()) {
    LOG_FATAL("Process GNSS: Could not open the GNSS file: " << gnss_file_path);
    exit(1);
  }

  struct RawFix {
    double ts;
    double lat;
    double lon;
    double h;
    Eigen::Vector3d pos_std;
    double roll;
    double pitch;
    double heading_deg;
    double geoid;
    bool has_orientation;
  };

  std::vector<RawFix> raw_fixes;
  std::string line, word;
  bool origin_set = false;
  bool detected_7_col = false;

  while (std::getline(gnss_file, line)) {
    // Replace comma and tab with space for uniform tokenization
    for (char &c : line) {
      if (c == ',' || c == '\t')
        c = ' ';
    }

    std::vector<std::string> row;
    std::stringstream ss(line);
    while (ss >> word) {
      row.push_back(word);
    }

    if (row.size() < 7)
      continue;

    try {
      RawFix fix;
      fix.ts = std::stod(row[0]);
      fix.lat = std::stod(row[1]);
      fix.lon = std::stod(row[2]);
      fix.h = std::stod(row[3]);
      if (row.size() >= 13) {
        // NovAtel / Inertial Explorer format (e.g. 32 cols):
        // Col 7: Roll, Col 8: Pitch, Col 9: Heading
        // Col 10, 11, 12: PosSTDN, PosSTDE, PosSTDD
        fix.roll = std::stod(row[7]);
        fix.pitch = std::stod(row[8]);
        fix.heading_deg = std::stod(row[9]);
        fix.pos_std = Eigen::Vector3d(std::stod(row[10]), std::stod(row[11]), std::stod(row[12]));
        fix.geoid = 0.0;
        fix.has_orientation = true;
      } else if (row.size() >= 11) {
        fix.pos_std = Eigen::Vector3d(std::stod(row[4]), std::stod(row[5]), std::stod(row[6]));
        fix.roll = std::stod(row[7]);
        fix.pitch = std::stod(row[8]);
        fix.heading_deg = std::stod(row[9]);
        fix.geoid = std::stod(row[10]);
        fix.has_orientation = true;
      } else {
        fix.pos_std = Eigen::Vector3d(std::stod(row[4]), std::stod(row[5]), std::stod(row[6]));
        fix.roll = 0.0;
        fix.pitch = 0.0;
        fix.heading_deg = 0.0;
        fix.geoid = 0.0;
        fix.has_orientation = false;
        detected_7_col = true;
      }
      raw_fixes.push_back(fix);
    } catch (...) {
      // Ignore header or malformed rows
    }
  }
  gnss_file.close();

  if (detected_7_col) {
    LOG_WARN("Process GNSS: 7-column format detected (missing orientation). "
             "Defaulting roll=0, pitch=0 and estimating heading from trajectory.");
  }

  double last_heading = 0.0;
  bool has_heading = false;

  for (size_t i = 0; i < raw_fixes.size(); ++i) {
    double heading = raw_fixes[i].heading_deg;

    if (!raw_fixes[i].has_orientation) {
      double dx = 0.0, dy = 0.0;
      if (i + 1 < raw_fixes.size()) {
        double x_next, y_next, x_curr, y_curr;
        WGS84toTWD97(raw_fixes[i + 1].lat, raw_fixes[i + 1].lon, x_next, y_next);
        WGS84toTWD97(raw_fixes[i].lat, raw_fixes[i].lon, x_curr, y_curr);
        dx = x_next - x_curr;
        dy = y_next - y_curr;
      } else if (i > 0) {
        double x_prev, y_prev, x_curr, y_curr;
        WGS84toTWD97(raw_fixes[i - 1].lat, raw_fixes[i - 1].lon, x_prev, y_prev);
        WGS84toTWD97(raw_fixes[i].lat, raw_fixes[i].lon, x_curr, y_curr);
        dx = x_curr - x_prev;
        dy = y_curr - y_prev;
      }

      double dist = std::hypot(dx, dy);
      if (dist > 0.1) {
        // TWD97: x is East, y is North.
        // Clockwise heading from North (North=0, East=90):
        heading = std::atan2(dx, dy) * 180.0 / M_PI;
        if (heading < 0.0)
          heading += 360.0;
        last_heading = heading;
        has_heading = true;
      } else if (has_heading) {
        heading = last_heading;
      } else {
        heading = 0.0;
      }
    }

    GNSSData data;
    if (convertRawFixToGNSSData(raw_fixes[i].ts, raw_fixes[i].lat, raw_fixes[i].lon,
                                raw_fixes[i].h, raw_fixes[i].roll, raw_fixes[i].pitch,
                                heading, raw_fixes[i].pos_std, raw_fixes[i].geoid,
                                local_origin, origin_set, data)) {
      gnss_data_list.push_back(data);
    }
  }

  LOG_INFO("Process GNSS: Successfully loaded " << gnss_data_list.size()
                                                << " GNSS records.");
}

GNSSData interpolateGnss(double t, const std::vector<GNSSData> &gnss_list) {
  if (gnss_list.empty()) return GNSSData();
  if (t <= gnss_list.front().timestamp) return gnss_list.front();
  if (t >= gnss_list.back().timestamp) return gnss_list.back();

  auto it = std::lower_bound(
      gnss_list.begin(), gnss_list.end(), t,
      [](const GNSSData &a, double val) { return a.timestamp < val; });

  const GNSSData &prev = *(it - 1);
  const GNSSData &next = *it;
  double dt = next.timestamp - prev.timestamp;
  double alpha = dt > 1e-9 ? (t - prev.timestamp) / dt : 0.0;

  GNSSData res;
  res.timestamp = t;
  res.lat = prev.lat + alpha * (next.lat - prev.lat);
  res.lon = prev.lon + alpha * (next.lon - prev.lon);
  res.alt = prev.alt + alpha * (next.alt - prev.alt);

  double twd_x = 0.0, twd_y = 0.0;
  WGS84toTWD97(res.lat, res.lon, twd_x, twd_y);
  res.twd97_x = twd_x;
  res.twd97_y = twd_y;
  res.twd97_z = res.alt;

  res.raw_roll = prev.raw_roll + alpha * (next.raw_roll - prev.raw_roll);
  res.raw_pitch = prev.raw_pitch + alpha * (next.raw_pitch - prev.raw_pitch);

  // Shortest angular difference for raw_yaw
  double d_yaw = next.raw_yaw - prev.raw_yaw;
  while (d_yaw > 180.0) d_yaw -= 360.0;
  while (d_yaw < -180.0) d_yaw += 360.0;
  res.raw_yaw = prev.raw_yaw + alpha * d_yaw;
  while (res.raw_yaw > 180.0) res.raw_yaw -= 360.0;
  while (res.raw_yaw < -180.0) res.raw_yaw += 360.0;

  res.position_std = (1.0 - alpha) * prev.position_std + alpha * next.position_std;
  res.geoidSep = (1.0 - alpha) * prev.geoidSep + alpha * next.geoidSep;

  if (prev.pose_6d.size() >= 6 && next.pose_6d.size() >= 6) {
    res.pose_6d = Eigen::VectorXd(6);
    res.pose_6d.head<3>() = (1.0 - alpha) * prev.pose_6d.head<3>() + alpha * next.pose_6d.head<3>();
    res.pose_6d[3] = (1.0 - alpha) * prev.pose_6d[3] + alpha * next.pose_6d[3];
    res.pose_6d[4] = (1.0 - alpha) * prev.pose_6d[4] + alpha * next.pose_6d[4];
    double d_enu_yaw = next.pose_6d[5] - prev.pose_6d[5];
    while (d_enu_yaw > 180.0) d_enu_yaw -= 360.0;
    while (d_enu_yaw < -180.0) d_enu_yaw += 360.0;
    double enu_yaw = prev.pose_6d[5] + alpha * d_enu_yaw;
    while (enu_yaw > 180.0) enu_yaw -= 360.0;
    while (enu_yaw < -180.0) enu_yaw += 360.0;
    res.pose_6d[5] = enu_yaw;
  }

  return res;
}

} // namespace lidar_dynamic_merge
