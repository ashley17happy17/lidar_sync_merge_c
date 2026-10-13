#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ ! -d "third_party" ]; then
    echo "[INFO] third_party folder not found. Running create_third_party.sh..."
    bash create_third_party.sh
fi

if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"
JOBS="${JOBS:-$(nproc)}"

cmake -S . -B build -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build build -j "${JOBS}"
