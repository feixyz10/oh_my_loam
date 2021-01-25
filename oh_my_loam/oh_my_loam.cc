#include "oh_my_loam.h"

#include <vector>

#include "common/pcl/pcl_utils.h"
#include "oh_my_loam/base/helper.h"
#include "oh_my_loam/extractor/extractor_VLP16.h"

namespace oh_my_loam {

namespace {
const double kPointMinDist = 0.1;
}  // namespace

bool OhMyLoam::Init() {
  config_ = common::YAMLConfig::Instance()->config();
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
  mapper_.reset(new Mapper);
  if (!mapper_->Init()) {
    AERROR << "Failed to initialize mapper";
    return false;
  }
  return true;
}

void OhMyLoam::Reset() {
  timestamp_last_ = timestamp_last_mapping_ = 0.0;
  extractor_->Reset();
  odometer_->Reset();
  mapper_->Reset();
  std::vector<common::Pose3d>().swap(poses_);
}

void OhMyLoam::Run(double timestamp,
                   const common::PointCloudConstPtr &cloud_in) {
  common::PointCloudPtr cloud(new common::PointCloud);
  RemoveOutliers(*cloud_in, cloud.get());
  std::vector<Feature> features;
  extractor_->Process(timestamp, cloud, &features);
  Pose3d pose;
  odometer_->Process(timestamp, features, &pose);
  poses_.emplace_back(pose);
  if (!IsMapping(timestamp)) return;
  mapper_->Process();
}

void OhMyLoam::RemoveOutliers(const common::PointCloud &cloud_in,
                              common::PointCloud *const cloud_out) const {
  common::RemovePoints<common::Point>(cloud_in, cloud_out, [&](const auto &pt) {
    return !common::IsFinite<common::Point>(pt) ||
           common::DistanceSquare<common::Point>(pt) <
               kPointMinDist * kPointMinDist;
  });
}

bool OhMyLoam::IsMapping(double timestamp) const {
  return std::abs(timestamp - timestamp_last_mapping_) >=
         config_["mapper_config"]["process_period"].as<double>();
}

}  // namespace oh_my_loam