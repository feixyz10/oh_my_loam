#pragma once

#include "common/geometry/pose3d.h"
#include "oh_my_loam/base/types.h"

namespace oh_my_loam {

using common::Pose3d;

template <typename PT>
void TransformPoint(const Pose3d &pose, const PT &pt_in, PT *const pt_out) {
  *pt_out = pt_in;
  Eigen::Vector3d pnt = pose * Eigen::Vector3d(pt_in.x, pt_in.y, pt_in.z);
  pt_out->x = pnt.x();
  pt_out->y = pnt.y();
  pt_out->z = pnt.z();
}

template <typename PT>
PT TransformPoint(const Pose3d &pose, const PT &pt_in) {
  PT pt_out;
  TransformPoint<PT>(pose, pt_in, &pt_out);
  return pt_out;
}

/**
 * @brief Transform a lidar point to the start of the scan
 *
 * @param pose Relative pose, end scan time w.r.t. start scan time
 */
void TransformToStart(const Pose3d &pose, const TPoint &pt_in,
                      TPoint *const pt_out);

TPoint TransformToStart(const Pose3d &pose, const TPoint &pt_in);

/**
 * @brief Transform a lidar point to the end of the scan
 *
 * @param pose Relative pose, end scan time w.r.t. start scan time
 */
void TransformToEnd(const Pose3d &pose, const TPoint &pt_in,
                    TPoint *const pt_out);

TPoint TransformToEnd(const Pose3d &pose, const TPoint &pt_in);

struct PointLinePair {
  TPoint pt;
  struct Line {
    TPoint pt1, pt2;
    Line() = default;
    Line(const TPoint &pt1, const TPoint &pt2) : pt1(pt1), pt2(pt2) {}
  };
  Line line;
  PointLinePair(const TPoint &pt, const Line &line) : pt(pt), line(line) {}
  PointLinePair(const TPoint &pt, const TPoint &pt1, const TPoint &pt2)
      : pt(pt), line(pt1, pt2) {}
};

struct PointPlanePair {
  TPoint pt;
  struct Plane {
    TPoint pt1, pt2, pt3;
    Plane() = default;
    Plane(const TPoint &pt1, const TPoint &pt2, const TPoint &pt3)
        : pt1(pt1), pt2(pt2), pt3(pt3) {}
  };
  Plane plane;
  PointPlanePair(const TPoint &pt, const Plane &plane) : pt(pt), plane(plane) {}
  PointPlanePair(const TPoint &pt, const TPoint &pt1, const TPoint &pt2,
                 const TPoint &pt3)
      : pt(pt), plane(pt1, pt2, pt3) {}
};

}  // namespace oh_my_loam