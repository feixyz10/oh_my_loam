#include "oh_my_loam.h"

#include "extractor/extractor_VLP16.h"

namespace oh_my_loam {

namespace {
const double kPointMinDist = 0.1;
}

bool OhMyLoam::Init() {
  YAML::Node config = Config::Instance()->config();
  extractor_.reset(new ExtractorVLP16);
  if (!extractor_->Init(config["extractor_config"])) {
    AERROR << "Failed to initialize extractor";
    return false;
  }
  odometry_.reset(new Odometry);
  if (!odometry_->Init(config["odometry_config"])) {
    AERROR << "Failed to initialize odometry";
    return false;
  }
  return true;
}

void OhMyLoam::Run(const PointCloud& cloud_in, double timestamp) {
  PointCloudPtr cloud(new PointCloud);
  RemoveOutliers(cloud_in, cloud.get());
  FeaturePoints feature_points;
  extractor_->Process(*cloud, &feature_points);
  Pose3D pose;
  odometry_->Process(feature_points, &pose);
  poses_.emplace_back(pose);
}

void OhMyLoam::RemoveOutliers(const PointCloud& cloud_in,
                              PointCloud* const cloud_out) const {
  RemoveNaNPoint<Point>(cloud_in, cloud_out);
  RemoveClosedPoints<Point>(*cloud_out, cloud_out, kPointMinDist);
}

}  // namespace oh_my_loam