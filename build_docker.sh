#!/bin/bash

echo "Build Docker"

sudo docker build --progress=plain -f ./Dockerfile -t lidar_dynamic_merge_c:noetic .

echo "Docker successfully build!"