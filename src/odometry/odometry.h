#pragma once

#include <pcl/kdtree/kdtree_flann.h>

#include <eigen3/Eigen/Dense>

#include "common.h"

namespace oh_my_loam {

float GetTime(const TPoint& pt);
int GetScanId(const TPoint& pt);

struct PointLinePair {
  Point pt;
  struct Line {
    Point pt1, pt2;
    Line(const Point& pt1, const Point& pt2) : pt1(pt1), pt2(pt2) {}
  };
  Line line;
  PointLinePair(const Point& pt, const Line& line) : pt(pt), line(line) {}
  double PointToLineDist() { return 0.0; }
};

struct PointPlanePair {
  Point pt;
  struct Plane {
    Point pt1, pt2, pt3;
    Plane(const Point& pt1, const Point& pt2, const Point& pt3)
        : pt1(pt1), pt2(pt2), pt3(pt3) {}
  };
  Plane plane;
  PointLinePair(const Point& pt, const Plane& plane) : pt(pt), plane(plane) {}
  double PointToPlaneDist() { return 0.0; }
};

// frame to frame lidar odometry
class Odometry {
 public:
  Odometry() = default;

  bool Init(const YAML::Node& config);

  void Process(const FeaturePoints& feature);

 protected:
  void UpdatePre(const FeaturePoints& feature);
  void AssociateCornPoints(const TPointCloud& sharp_corn_pts_curr,
                           std::vector<PointLinePair>* const pairs) const;

  void AssociateSurfPoints(const TPointCloud& flat_surf_pts_curr,
                           std::vector<PointPlanePair>* const pairs) const;

  YAML::Node config_;

  bool is_initialized = false;

  TPointCloudPtr surf_pts_pre_;
  TPointCloudPtr corn_pts_pre_;

  pcl::KdTreeFLANN<TPoint>::Ptr kdtree_surf_pts(new pcl::KdTreeFLANN<TPoint>);
  pcl::KdTreeFLANN<TPoint>::Ptr kdtree_corn_pts(new pcl::KdTreeFLANN<TPoint>);
};

}  // namespace oh_my_loam
