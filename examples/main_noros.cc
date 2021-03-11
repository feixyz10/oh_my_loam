#include <pcl/io/pcd_io.h>

#include <chrono>
#include <filesystem>
#include <functional>
#include <thread>

#include "common/common.h"
#include "oh_my_loam/oh_my_loam.h"

using namespace common;
using namespace oh_my_loam;
namespace fs = std::filesystem;

void PointCloudHandler(const PointCloudConstPtr &cloud, OhMyLoam *const slam);

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cerr << "\033[1m\033[31mConfiguration file and input path should be "
                 "specified!\033[m"
              << std::endl;
    return -1;
  }
  // config
  YAMLConfig::Instance()->Init(argv[1]);
  bool is_log_to_file = YAMLConfig::Instance()->Get<bool>("log_to_file");
  std::string log_path = YAMLConfig::Instance()->Get<std::string>("log_path");
  std::string lidar = YAMLConfig::Instance()->Get<std::string>("lidar");
  // logging
  InitG3Logging(is_log_to_file, "oh_my_loam_" + lidar, log_path);
  AUSER << "LOAM start..., lidar = " << lidar;
  // SLAM system
  OhMyLoam slam;
  if (!slam.Init()) {
    AFATAL << "Failed to initilize slam system.";
  }
  // get input point cloud file names
  ACHECK(fs::exists(argv[2])) << "Directory not exist: " << argv[2];
  std::vector<fs::path> cloud_paths;
  for (auto &it : fs::directory_iterator(argv[2])) {
    if (fs::is_regular_file(it.path())) cloud_paths.push_back(it.path());
  }
  AWARN_IF(cloud_paths.empty())
      << "No point cloud file in directory: " << argv[2];
  AINFO_IF(!cloud_paths.empty())
      << "There are " << cloud_paths.size() << " point clouds in total";
  std::sort(cloud_paths.begin(), cloud_paths.end());
  // load point cloud and process
  for (auto &path : cloud_paths) {
    PointCloudPtr cloud(new PointCloud);
    pcl::io::loadPCDFile(path.string(), *cloud);
    PointCloudHandler(cloud, &slam);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10 Hz
  }
  return 0;
}

void PointCloudHandler(const PointCloudConstPtr &cloud, OhMyLoam *const slam) {
  auto millisecs = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  double timestamp = millisecs.count() / 1000.0;
  static size_t frame_id = 0;
  AINFO << "Ohmyloam: frame_id = " << ++frame_id
        << ", timestamp = " << FMT_TIMESTAMP(timestamp)
        << ", point_number = " << cloud->size();
  slam->Run(timestamp, cloud);
}