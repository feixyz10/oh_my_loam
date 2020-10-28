#pragma once

#include "common.h"

namespace oh_my_loam {

float GetTime(const TPoint& pt);

int GetScanId(const TPoint& pt);

struct PointLinePair {
  TPoint pt;
  struct Line {
    TPoint pt1, pt2;
    Line() = default;
    Line(const TPoint& pt1, const TPoint& pt2) : pt1(pt1), pt2(pt2) {}
  };
  Line line;
  PointLinePair(const TPoint& pt, const Line& line) : pt(pt), line(line) {}
  double DistPointToLine() const;
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
  double DistPointToPlane() const;
};

}  // oh_my_loam