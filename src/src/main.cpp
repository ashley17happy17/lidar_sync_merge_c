#include "lidar_dynamicmerge_c/dynamic_merge_node.hpp"
#include "lidar_dynamicmerge_c/logger.hpp"
#include <iostream>
#include <pcl/console/print.h>
#include <string>
#include <sys/stat.h>
#include <vector>
#include <yaml-cpp/yaml.h>

std::string extractLogFileName(const std::string &gnss_file) {
  size_t last_slash = gnss_file.find_last_of("/\\");
  if (last_slash == std::string::npos)
    return "default.log";
  std::string parent_dir = gnss_file.substr(0, last_slash);
  size_t second_last_slash = parent_dir.find_last_of("/\\");
  std::string sub_parent_dir = parent_dir.substr(0, second_last_slash);
  size_t third_last_slash = sub_parent_dir.find_last_of("/\\");
  if (third_last_slash == std::string::npos)
    return parent_dir + ".log";
  return sub_parent_dir.substr(third_last_slash + 1) + ".log";
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <config_file>\n";
    return 1;
  }
  std::string config_file = argv[1];

  std::string log_filename = "dynamic_merge.log";
  try {
    YAML::Node config = YAML::LoadFile(config_file);
    if (config["gnss_file"]) {
      log_filename = extractLogFileName(config["gnss_file"].as<std::string>());
    }

    mkdir("log", 0777);
    lidar_dynamic_merge::Logger::getInstance().init("log/" + log_filename);

    if (config["log_level"]) {
      std::string level_str = config["log_level"].as<std::string>();
      if (level_str == "DEBUG")
        lidar_dynamic_merge::Logger::getInstance().setLevel(
            lidar_dynamic_merge::LogLevel::DEBUG);
      else if (level_str == "INFO")
        lidar_dynamic_merge::Logger::getInstance().setLevel(
            lidar_dynamic_merge::LogLevel::INFO);
      else if (level_str == "WARN")
        lidar_dynamic_merge::Logger::getInstance().setLevel(
            lidar_dynamic_merge::LogLevel::WARN);
      else if (level_str == "ERROR")
        lidar_dynamic_merge::Logger::getInstance().setLevel(
            lidar_dynamic_merge::LogLevel::ERROR);
      else if (level_str == "FATAL")
        lidar_dynamic_merge::Logger::getInstance().setLevel(
            lidar_dynamic_merge::LogLevel::FATAL);
    }
  } catch (...) {
    mkdir("log", 0777);
    lidar_dynamic_merge::Logger::getInstance().init("log/" + log_filename);
  }

  // Suppress PCL warnings (like "Leaf size is too small for")
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  try {
    auto start = std::chrono::high_resolution_clock::now();
    lidar_dynamic_merge::DynamicMergeNode merge(config_file);
    merge.run();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    LOG_INFO("Main: Total execution time: " << elapsed.count() << " seconds");
  } catch (const YAML::BadFile &e) {
    LOG_FATAL("Main: Config file not found.");
  } catch (const YAML::ParserException &e) {
    LOG_FATAL("Main: Parse Error: " << e.what());
  }
  return 0;
}