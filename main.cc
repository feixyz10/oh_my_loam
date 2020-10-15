#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include <functional>

#include "log.h"
#include "oh_my_loam.h"

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg,
                       oh_my_loam::OhMyLoam* const slam);

int main(int argc, char* argv[]) {
  // configurations
  YAML::Node config = YAML::LoadFile("./config/config.yaml");

  // logging
  g3::InitG3Logging<true>("oh_my_loam", ".log");
  AWARN << config["lidar"].as<std::string>();

  // SLAM system
  oh_my_loam::OhMyLoam slam;
  slam.Init(config["feature_points_extractor_config"]);

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
                       oh_my_loam::OhMyLoam* const slam) {
  oh_my_loam::PointCloud cloud;
  pcl::fromROSMsg(*msg, cloud);
  AINFO << "Point num = " << cloud.size()
        << ", ts = " << msg->header.stamp.toSec();
  slam->Run(cloud, 0.0);
}