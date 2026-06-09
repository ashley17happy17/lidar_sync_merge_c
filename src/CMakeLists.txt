cmake_minimum_required(VERSION 3.14)
project(lidar_dynamic_merge_c)

# Compile options
add_compile_options(-std=c++17 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
)

# Find PCL
find_package(PCL REQUIRED)
# Find Eigen3
find_package(Eigen3 REQUIRED)
# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_OPENMP ON CACHE BOOL "" FORCE)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs geometry_msgs pcl_ros pcl_conversions cv_bridge
  DEPENDS PCL Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_subdirectory(third_party/lidar_utils_lib/src)



# Node Executable
add_executable(${PROJECT_NAME}_node src/main.cpp src/lidar_dynamic_merge.cpp)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  lidar_utils
  yaml-cpp
)

