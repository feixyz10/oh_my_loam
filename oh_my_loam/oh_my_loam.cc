#include "oh_my_loam.h"

#include <vector>

#include "common/common.h"
#include "common/pcl/pcl_utils.h"
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
  AWARN << "OhMySlam RESET";
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
  common::Pose3d pose_curr2odom;
  odometer_->Process(timestamp, features, &pose_curr2odom);
  common::Pose3d pose_curr2map;
  const auto &cloud_corn = odometer_->GetCloudCorn();
  const auto &cloud_surf = odometer_->GetCloudSurf();
  mapper_->Process(timestamp, cloud_corn, cloud_surf, pose_curr2odom,
                   &pose_curr2map);
  poses_curr2odom_.push_back(pose_curr2odom);
  poses_curr2world_.push_back(pose_curr2map);
  if (is_vis_) Visualize(timestamp);
}

void OhMyLoam::Visualize(double timestamp) {}

void OhMyLoam::RemoveOutliers(const common::PointCloud &cloud_in,
                              common::PointCloud *const cloud_out) const {
  common::RemovePoints<common::Point>(
      cloud_in, cloud_out, [&](const common::Point &pt) {
        return !common::IsFinite(pt) ||
               common::DistanceSquare(pt) < kPointMinDist * kPointMinDist;
      });
}

}  // namespace oh_my_loam