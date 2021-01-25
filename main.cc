#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <filesystem>
#include <functional>

#include "common/common.h"
#include "oh_my_loam/oh_my_loam.h"

using namespace common;
using namespace oh_my_loam;

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg,
                       OhMyLoam *const slam);

int main(int argc, char *argv[]) {
  // config
  YAMLConfig::Instance()->Init(argv[1]);
  bool log_to_file = YAMLConfig::Instance()->Get<bool>("log_to_file");
  std::string log_path = YAMLConfig::Instance()->Get<std::string>("log_path");
  std::string lidar = YAMLConfig::Instance()->Get<std::string>("lidar");

  // logging
  InitG3Logging(log_to_file, "oh_my_loam_" + lidar, log_path);
  AUSER << "LOAM start..., lidar = " << lidar;

  // SLAM system
  OhMyLoam slam;
  if (!slam.Init()) {
    AFATAL << "Failed to initilize slam system.";
  }

  // ros
  ros::init(argc, argv, "oh_my_loam");
  ros::NodeHandle nh;
  ros::Subscriber sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_points", 100,
      std::bind(PointCloudHandler, std::placeholders::_1, &slam));
  ros::spin();

  return 0;
}

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg,
                       OhMyLoam *const slam) {
  PointCloudPtr cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *cloud);
  double timestamp = msg->header.stamp.toSec();
  static size_t frame_id = 0;
  AINFO << "Ohmyloam: frame_id = " << ++frame_id
        << ", timestamp = " << FMT_TIMESTAMP(timestamp);
  slam->Run(timestamp, cloud);
}