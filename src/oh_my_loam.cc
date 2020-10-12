#include "oh_my_loam.h"

namespace oh_my_loam {

bool OhMyLoam::Init() {
  is_vis_ = false;
  feature_extractor_.reset(new FeatureExtractorVLP16);
  if (is_vis_) {
    visualizer_.reset(new FeaturePointsVisualizer);
  }
  return true;
}

void OhMyLoam::Run(const PointCloud& cloud, double timestamp) {
  pcl::PointCloud<pcl::PointXYZ> cl;
  std::shared_ptr<FeaturePoints> feature_pts(new FeaturePoints);
  feature_extractor_->Extract(cloud, feature_pts.get());
  if (is_vis_) Visualize(cloud, feature_pts, timestamp);
}

void OhMyLoam::Visualize(
    const PointCloud& cloud,
    const std::shared_ptr<const FeaturePoints>& feature_pts, double timestamp) {
  std::shared_ptr<FeaturePointsVisFrame> frame;
  frame->timestamp = timestamp;
  frame->cloud = cloud.makeShared();
  frame->feature_pts = feature_pts;
  visualizer_->Render(frame);
}

}  // oh_my_loam