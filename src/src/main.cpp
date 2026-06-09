#include "lidar_dynamic_merge.hpp"
#include <iostream>
#include <pcl/console/print.h>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  // Suppress PCL warnings (like "Leaf size is too small for")
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  try {
    if (argc != 2) {
      std::cout << "Usage: " << argv[0] << " <config_file>" << std::endl;
      return 1;
    }
    std::string config_file = argv[1];
    lidar_dynamic_merge::LiDARDynamicMerge merge(config_file);
    merge.run();
  } catch (const YAML::BadFile &e) {
    std::cerr << "Error: Config file not found.\n";
  } catch (const YAML::ParserException &e) {
    std::cerr << "Parse Error: " << e.what() << "\n";
  }
  return 0;
}