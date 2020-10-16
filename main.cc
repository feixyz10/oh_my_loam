#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <functional>

#include "common/common.h"
#include "src/oh_my_loam.h"

using namespace oh_my_loam;

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg,
                       OhMyLoam* const slam);

int main(int argc, char* argv[]) {
  // config
  Config::Instance()->SetConfigFile("configs/config.yaml");
  bool log_to_file = Config::Instance()->Get<bool>("log_to_file");
  std::string log_path = Config::Instance()->Get<std::string>("log_path");
  std::string lidar = Config::Instance()->Get<std::string>("lidar");

  // logging
  g3::InitG3Logging(log_to_file, "oh_my_loam_" + lidar, log_path);
  AINFO << "Lidar: " << lidar;

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

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg,
                       OhMyLoam* const slam) {
  PointCloud cloud;
  pcl::fromROSMsg(*msg, cloud);
  ADEBUG << "Point num = " << cloud.size()
         << ", ts = " << LOG_TIMESTAMP(msg->header.stamp.toSec());
  slam->Run(cloud, 0.0);
}