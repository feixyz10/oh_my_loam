#pragma once

#include "common.h"

namespace oh_my_loam {

inline float GetTime(const TPoint& pt) {
  return pt.time - static_cast<int>(pt.time);
}

inline int GetScanId(const TPoint& pt) { return static_cast<int>(pt.time); }

/**
 * @brief Transform a lidar point to the start of the scan
 *
 * @param pose Relative pose, end scan time w.r.t. start scan time
 * @param time Point time relative to the start time of the scan, \in [0, 1]
 */
void TransformToStart(const Pose3D& pose, const TPoint& pt_in,
                      TPoint* const pt_out) {
  Pose3D pose_interp = Pose3D().Interpolate(pose, GetTime(pt_in));
  *pt_out = pose_interp * pt_in;
}

/**
 * @brief Transform a lidar point to the end of the scan
 *
 * @param pose Relative pose, end scan time w.r.t. start scan time
 * @param time Point time relative to the start time of the scan, \in [0, 1]
 */
void TransformToEnd(const Pose3D& pose, const TPoint& pt_in,
                    TPoint* const pt_out) {
  TransformToStart(pose, pt_in, pt_out);
  *pt_out = pose.Inv() * (*pt_out);
}

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