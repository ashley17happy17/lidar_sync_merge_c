#include "lidar_dynamicmerge_c/dynamic_merge_node.hpp"
#include "lidar_dynamicmerge_c/logger.hpp"
#include <filesystem>
#include <iostream>
#include <pcl/console/print.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace {
constexpr const char *kFallbackLogDir = "log";
}

std::string extractLogFileName(const std::string &gnss_file) {
  size_t last_slash = gnss_file.find_last_of("/\\");
  if (last_slash == std::string::npos)
    return "default.log";
  std::string parent_dir = gnss_file.substr(0, last_slash);
  size_t second_last_slash = parent_dir.find_last_of("/\\");
  std::string sub_parent_dir = parent_dir.substr(0, second_last_slash);
  size_t third_last_slash = sub_parent_dir.find_last_of("/\\");
  std::string sub_sub_parent_dir = parent_dir.substr(0, third_last_slash);
  size_t fourth_last_slash = sub_sub_parent_dir.find_last_of("/\\");
  if (fourth_last_slash == std::string::npos)
    return parent_dir + ".log";
  return sub_sub_parent_dir.substr(fourth_last_slash + 1) + "_" + sub_parent_dir.substr(third_last_slash + 1) + ".log";
}

std::string extractLogDir(const std::string &gnss_file) {
  std::filesystem::path gnss_dir =
      std::filesystem::path(gnss_file).parent_path();
  if (gnss_dir.empty())
    return kFallbackLogDir;
  std::filesystem::path record_dir = gnss_dir.parent_path();
  if (record_dir.empty() || record_dir == record_dir.root_path())
    return kFallbackLogDir;
  return (record_dir / "Logs").string();
}

// Creates log_dir and returns the full log file path, falling back to the
// local "log" folder when the derived directory is not writable.
std::string prepareLogPath(const std::string &log_dir,
                           const std::string &log_filename) {
  std::error_code ec;
  std::filesystem::create_directories(log_dir, ec);
  if (ec) {
    std::cerr << "Failed to create log directory: " << log_dir << " ("
              << ec.message() << "), falling back to " << kFallbackLogDir
              << std::endl;
    std::filesystem::create_directories(kFallbackLogDir, ec);
    return std::string(kFallbackLogDir) + "/" + log_filename;
  }
  return log_dir + "/" + log_filename;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <config_file>\n";
    return 1;
  }
  std::string config_file = argv[1];

  std::string log_filename = "dynamic_merge.log";
  std::string log_dir = kFallbackLogDir;
  try {
    YAML::Node config = YAML::LoadFile(config_file);
    if (config["gnss_file"]) {
      std::string gnss_file = config["gnss_file"].as<std::string>();
      log_dir = extractLogDir(gnss_file);
    }

    lidar_dynamic_merge::Logger::getInstance().init(
        prepareLogPath(log_dir, log_filename));

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

    if (config["console_output"]) {
      lidar_dynamic_merge::Logger::getInstance().setConsoleOutput(
          config["console_output"].as<bool>());
    }
  } catch (...) {
    lidar_dynamic_merge::Logger::getInstance().init(
        prepareLogPath(log_dir, log_filename));
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
