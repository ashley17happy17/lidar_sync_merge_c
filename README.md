# LiDAR_DynamicMerge_C

## Introduction
`LiDAR_DynamicMerge_C` is a high-performance C++ application that synchronizes and merges point cloud scans from multiple LiDAR sensors and distinct timestamps into a single, motion-compensated point cloud.

The application supports two input modes:
1. **File Mode (Default)**: Ingests datasets from the filesystem (GNSS CSV, IMU TXT, ODOM CSV, and per-LiDAR directories of `.pcd` / `.las` files).
2. **ROS 1 Bag Mode**: Ingests multi-sensor data directly from a ROS 1 `.bag` file (`sensor_msgs/PointCloud2`, `sensor_msgs/Imu`, `nav_msgs/Odometry` / `geometry_msgs/Twist`, and GNSS fix/odom topics).

Output merged point clouds are saved directly to disk as PCD or LAS files.

---

## Project Architecture

The codebase is organized into modular layers:
- **`FileLoader`**: Reads CSV, TXT, and PCD/LAS folders from disk.
- **`BagLoader`**: Manages ROS 1 `.bag` queries, on-demand point cloud retrieval, and per-point timestamp unpacking (for Ouster and Velodyne) with thread-safe OpenMP execution.
- **`gnss_processor`**: Reusable geodetic projection (WGS84 $\rightarrow$ TWD97 TM2 $\rightarrow$ Local Cartesian origin) and compass heading to counter-clockwise ENU yaw normalization.
- **`time_sync` (`synchronizeFrames`)**: Multi-sensor temporal synchronizer matching GNSS trajectory epochs with multi-LiDAR frame timestamps.
- **`DynamicMergeNode`**: Coordinates the pipeline, executes OpenMP parallel processing (crop, denoise, motion compensation/deskew, direct georeferencing, voxel grid merging), and saves output point clouds.

---

## Build

The project requires PCL, Eigen3, yaml-cpp, PDAL, and ROS 1 Noetic (`roscpp`, `rosbag`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`).

### Building with Docker (Recommended)
```bash
# Build and enter the container
bash run_docker.sh

# Inside container:
cd /root/ws/src
bash build.sh
```

### Building on Host (with ROS 1 Noetic installed)
```bash
source /opt/ros/noetic/setup.bash
bash build.sh
```

Equivalent manual CMake commands:
```bash
source /opt/ros/noetic/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j "$(nproc)"
```

---

## Run

1. Edit or configure your parameter YAML file (e.g. `config/params_c_calib.yaml`).
2. Run the application:
```bash
./build/lidar_dynamic_merge_c_node config/params_c_calib.yaml
```
Or via script:
```bash
bash run.sh
```

---

## Configuration (`params_c_calib.yaml`)

```yaml
## Parameters for lidar_dynamic_merge_c

log_level: "INFO"         # Supported levels: DEBUG, INFO, WARN, ERROR, FATAL
console_output: false     # Set to false to disable printing to terminal (errors will still print)

# -------------------------------------------------------------------
# Input Source Switching
# -------------------------------------------------------------------
# Set bag_file to run in ROS 1 Bag Mode.
# Leave as empty string "" to run in File Mode (CSV/TXT/PCD directories):
bag_file: ""

# In BAG mode: these are ROS topic names.
# In FILE mode: these are file/folder paths.
gnss_source: "/root/ws/pcd/20260918/rec01/VTS/20260918113847_gnss_full.csv"
gnss_std_thres: 500.0            # unit: meter
gnss_freq: 10                    # unit: Hz

lidar_hz: 10                     # unit: Hz, max allowed GNSS-to-LiDAR time gap during sync is 1/lidar_hz
lidar_time_offset: 0.0           # unit: second, clock offset ADDED to every LiDAR timestamp

imu_source: "/root/ws/pcd/20260918/rec01/VTS/VTS_adr_20260918113847_imu.txt"
imu_freq: 100                    # unit: Hz

odom_source: "/root/ws/pcd/20260918/rec01/VTS/can_VehSpeed_20260918113847.csv"
odom_speed_unit: "kph"           # unit of speed: "mps" (m/s) or "kph" (km/h)

# Motion-compensation translation source:
#   GNSS    : GNSS position delta
#   ODOM    : integrated odometer velocity
#   IMU_ACC : double-integrated IMU accel
#   LEGACY  : legacy method - per-point GNSS pose interpolation, no IMU
motion_method: "ODOM"

# -------------------------------------------------------------------
# LiDAR Parameters
# -------------------------------------------------------------------
# In BAG mode: lidarX_source is the sensor_msgs/PointCloud2 topic name.
# In FILE mode: lidarX_source is the folder path containing PCD/LAS files.

lidar1_source: "/root/ws/pcd/20260918/rec01/PCD/top/"
lidar1_format: "PCD_BINARY"
lidar1_type: "OUSTER_OS1_128"
lidar1_la: [0.467, 0.267, -0.396]
lidar1_bs: [-2.131, 0.677, -89.420]

lidar2_source: "/root/ws/pcd/20260918/rec01/PCD/rear_left/"
lidar2_format: "PCD_BINARY"
lidar2_type: "VELODYNE_VLP16"
lidar2_la: [-1.013, -0.636, 0.628]
lidar2_bs: [-0.020, 1.297, 5.778]

lidar3_source: "/root/ws/pcd/20260918/rec01/PCD/rear_right/"
lidar3_format: "PCD_BINARY"
lidar3_type: "VELODYNE_VLP16"
lidar3_la: [-0.939, 1.123, 0.682]
lidar3_bs: [-0.858, 4.668, 2.201]

lidar4_source: "/root/ws/pcd/20260918/rec01/PCD/front_right/"
lidar4_format: "PCD_BINARY"
lidar4_type: "VELODYNE_VLP16"
lidar4_la: [3.136, 1.114, 0.704]
lidar4_bs: [0.602, 2.361, -7.354]

lidar5_source: "/root/ws/pcd/20260918/rec01/PCD/front_left/"
lidar5_format: "PCD_BINARY"
lidar5_type: "VELODYNE_VLP16"
lidar5_la: [3.163, -0.700, 0.630]
lidar5_bs: [-0.680, 2.299, -4.954]

# -------------------------------------------------------------------
# Cloud Filtering Parameters
# -------------------------------------------------------------------
crop_min_bound: [0.0, 0.0, 0.0]
crop_max_bound: [100.0, 100.0, 100.0]

denoise_radius: 0.01
denoise_epsilon: 0.1

ds_voxel_size: 0.01
motion_enable: true
merge_voxel_size: 0.01

# -------------------------------------------------------------------
# Output Parameters
# -------------------------------------------------------------------
out_lidar: 1
out_source: "/root/ws/pcd/20260918/rec01/Merge/"
out_format: "PCD_BINARY"
out_la: [0.0, 0.0, 0.0]
out_bs: [0.0, 0.0, 0.0]
```