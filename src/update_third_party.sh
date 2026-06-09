#!/bin/bash

cd third_party/lidar_utils_lib
git fetch origin
# Insert the username and password
git reset --hard origin/develop
cd ../..