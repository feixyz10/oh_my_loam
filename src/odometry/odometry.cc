#include "odometry.h"

namespace oh_my_loam {

bool Odometry::Init(const YAML::Node& config) {
  config_ = config;
  return true;
}

void Odometry::Process(const FeaturePoints& feature) {
  if (!is_initialized) {
    is_initialized = true;
    UpdatePre(feature);
    return;
  }
  AssociateCornPoints();
  AssociateSurfPoints();

  UpdatePre(feature);
}

void Odometry::AssociateCornPoints(
    const TPointCloud& sharp_corn_pts_curr,
    std::vector<PointLinePair>* const pairs) const {
  for (const pt : sharp_corn_pts_curr) {
    Point p1, p2;

    pairs->emplace_back(pt, {p1, p2});
  }
}

void Odometry::AssociateSurfPoints(
    const TPointCloud& flat_surf_pts_curr,
    std::vector<PointPlanePair>* const pairs) const {
  for (const pt : flat_surf_pts_curr) {
    Point p1, p2, p3;
    pairs->emplace_back(pt, {p1, p2, p3});
  }
}

void Odometry::UpdatePre(const FeaturePoints& feature) {
  surf_pts_pre_ = feature.less_flat_surf_pts;
  corn_pts_pre_ = feature.less_sharp_corner_pts;
  kdtree_surf_pts_->setInputCloud(surf_pts_pre_);
  kdtree_corn_pts_->setInputCloud(corn_pts_pre_);
}

float GetTime(const TPoint& pt) { return pt.time - GetScanId(pt); }

int GetScanId(const TPoint& pt) { return static_cast<int>(pt.time); }

}  // namespace oh_my_loam
