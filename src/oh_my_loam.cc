#include "oh_my_loam.h"
#include "feature_points_extractor/feature_points_extractor_VLP16.h"

namespace oh_my_loam {

bool OhMyLoam::Init() {
  YAML::Node config = Config::Instance()->config();
  feature_extractor_.reset(new FeaturePointsExtractorVLP16);
  if (!feature_extractor_->Init(config["feature_extractor_config"])) {
    AERROR << "Failed to initialize feature points extractor";
    return false;
  }
  return true;
}

void OhMyLoam::Run(const PointCloud& cloud, double timestamp) {
  FeaturePoints feature_pts;
  feature_extractor_->Extract(cloud, &feature_pts);
}

}  // namespace oh_my_loam