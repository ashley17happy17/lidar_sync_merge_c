cmake_minimum_required(VERSION 3.14)
project(lidar_dynamic_merge_c VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# Compile options
add_compile_options(-O3 -Wall)

# Find PCL
find_package(PCL REQUIRED)
# Find Eigen3
find_package(Eigen3 REQUIRED)
# Find yaml-cpp
find_package(yaml-cpp REQUIRED)
# Find PDAL
find_package(PDAL REQUIRED)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_OPENMP ON CACHE BOOL "" FORCE)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# yaml-cpp exports an imported target on newer releases only
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

add_subdirectory(third_party/lidar_utils_lib/src)

# Node Executable
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/dynamic_merge_node.cpp
  src/gnss_processor.cpp
  src/config_loader.cpp
  src/time_sync.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PDAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${PDAL_LIBRARIES}
  lidar_utils
  ${YAML_CPP_TARGET}
)

# Drop the executable straight into the build root so run.sh can find it
set_target_properties(${PROJECT_NAME}_node PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION bin)
