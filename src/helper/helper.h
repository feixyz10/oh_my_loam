#pragma once

#include "common.h"

namespace oh_my_loam {

inline float GetTime(const TPoint& pt) {
  return pt.time - static_cast<int>(pt.time);
}

inline int GetScanId(const TPoint& pt) { return static_cast<int>(pt.time); }

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