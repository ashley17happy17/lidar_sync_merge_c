#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

CONFIG_FILE="${1:-config/params_c_calib_bag.yaml}"
echo "[INFO] Running LiDAR_DynamicMerge_C with config: $CONFIG_FILE"
time ./build/lidar_dynamic_merge_c_node "$CONFIG_FILE"

