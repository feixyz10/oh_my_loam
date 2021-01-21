#include "oh_my_loam.h"

#include "common/pcl/pcl_utils.h"
#include "oh_my_loam/extractor/extractor_VLP16.h"

namespace oh_my_loam {

namespace {
const double kPointMinDist = 0.1;
using namespace common;
}  // namespace

bool OhMyLoam::Init() {
  YAML::Node config = YAMLConfig::Instance()->config();
  extractor_.reset(new ExtractorVLP16);
  if (!extractor_->Init()) {
    AERROR << "Failed to initialize extractor";
    return false;
  }
  odometer_.reset(new Odometer);
  if (!odometer_->Init()) {
    AERROR << "Failed to initialize odometer";
    return false;
  }
  // mapper_.reset(new Mapper);
  // if (!mapper_->Init()) {
  //   AERROR << "Failed to initialize mapper";
  //   return false;
  // }
  return true;
}

void OhMyLoam::Run(double timestamp, const PointCloudConstPtr& cloud_in) {
  PointCloudPtr cloud(new PointCloud);
  RemoveOutliers(*cloud_in, cloud.get());
  std::vector<Feature> features;
  extractor_->Process(timestamp, cloud, &features);
  Pose3d pose;
  odometer_->Process(timestamp, features, &pose);
  poses_.emplace_back(pose);
}

void OhMyLoam::RemoveOutliers(const PointCloud& cloud_in,
                              PointCloud* const cloud_out) const {
  RemovePoints<Point>(cloud_in, cloud_out, [&](const Point& pt) {
    return !IsFinite<Point>(pt) ||
           DistanceSqure<Point>(pt) < kPointMinDist * kPointMinDist;
  });
}

}  // namespace oh_my_loam