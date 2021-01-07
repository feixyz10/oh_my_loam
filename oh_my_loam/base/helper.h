#pragma once

#include "common/geometry/pose.h"
#include "common/pcl/pcl_types.h"

namespace oh_my_loam {

using common::TPoint;
using common::Pose3D;
using common::Trans3d;

inline float GetTime(const TPoint& pt) { return pt.time - std::floor(pt.time); }

inline int GetScanId(const TPoint& pt) { return static_cast<int>(pt.time); }

template <typename PointType>
void TransformPoint(const Pose3D& pose, const PointType& pt_in,
                    PointType* const pt_out) {
  *pt_out = pt_in;
  Eigen::Vector3d pnt = pose * Eigen::Vector3d(pt_in.x, pt_in.y, pt_in.z);
  pt_out->x = pnt.x();
  pt_out->y = pnt.y();
  pt_out->z = pnt.z();
}

/**
 * @brief Transform a lidar point to the start of the scan
 *
 * @param pose Relative pose, end scan time w.r.t. start scan time
 */
void TransformToStart(const Pose3D& pose, const TPoint& pt_in,
                      TPoint* const pt_out);
/**
 * @brief Transform a lidar point to the end of the scan
 *
 * @param pose Relative pose, end scan time w.r.t. start scan time
 */
void TransformToEnd(const Pose3D& pose, const TPoint& pt_in,
                    TPoint* const pt_out);

struct PointLinePair {
  TPoint pt;
  struct Line {
    TPoint pt1, pt2;
    Line() = default;
    Line(const TPoint& pt1, const TPoint& pt2) : pt1(pt1), pt2(pt2) {}
  };
  Line line;
  PointLinePair(const TPoint& pt, const Line& line) : pt(pt), line(line) {}
  PointLinePair(const TPoint& pt, const TPoint& pt1, const TPoint& pt2)
      : pt(pt), line(pt1, pt2) {}
};

struct PointPlanePair {
  TPoint pt;
  struct Plane {
    TPoint pt1, pt2, pt3;
    Plane() = default;
    Plane(const TPoint& pt1, const TPoint& pt2, const TPoint& pt3)
        : pt1(pt1), pt2(pt2), pt3(pt3) {}
  };
  Plane plane;
  PointPlanePair(const TPoint& pt, const Plane& plane) : pt(pt), plane(plane) {}
  PointPlanePair(const TPoint& pt, const TPoint& pt1, const TPoint& pt2,
                 const TPoint& pt3)
      : pt(pt), plane(pt1, pt2, pt3) {}
};

}  // namespace oh_my_loam