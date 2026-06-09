#!/bin/bash
cd /root/catkin_ws/src
if [ ! -d "third_party" ]; then
    echo "[INFO] third_party folder not found. Running create_third_party.sh..."
    bash create_third_party.sh
fi

cd /root/catkin_ws
catkin_make -j20 -DCMAKE_BUILD_TYPE=RelWithDebInfo
cd ./src