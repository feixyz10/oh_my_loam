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
  extractor_->Reset();
  odometer_->Reset();
  mapper_->Reset();
}

void OhMyLoam::Run(double timestamp,
                   const common::PointCloudConstPtr &cloud_in) {
  common::PointCloudPtr cloud(new common::PointCloud);
  RemoveOutliers(*cloud_in, cloud.get());
  std::vector<Feature> features;
  extractor_->Process(timestamp, cloud, &features);
  FusionOdometryMapping();
  auto pose_odom =
      poses_curr2world_.empty() ? TimePose() : poses_curr2world_.back();
  odometer_->Process(timestamp, features, &pose_odom.pose);
  poses_curr2world_.push_back(pose_odom);
  const auto &cloud_corn = odometer_->cloud_corn();
  const auto &cloud_surf = odometer_->cloud_surf();

  if (!pose_mapping_updated_) return;
  mapping_thread_.reset(new std::thread(&OhMyLoam::MappingProcess, this,
                                        timestamp, cloud_corn, cloud_surf));
  if (mapping_thread_->joinable()) mapping_thread_->detach();
}

void OhMyLoam::FusionOdometryMapping() {
  std::lock_guard<std::mutex> lock(mutex_);
  TimePose pose_m = pose_mapping_;
  pose_mapping_updated_ = false;
  for (;;) {
  }
}

void OhMyLoam::Visualize(double timestamp) {}

void OhMyLoam::RemoveOutliers(const common::PointCloud &cloud_in,
                              common::PointCloud *const cloud_out) const {
  common::RemovePoints<common::Point>(cloud_in, cloud_out, [&](const auto &pt) {
    return !common::IsFinite<common::Point>(pt) ||
           common::DistanceSquare<common::Point>(pt) <
               kPointMinDist * kPointMinDist;
  });
}

}  // namespace oh_my_loam