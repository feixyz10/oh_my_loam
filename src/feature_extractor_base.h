#pragma once

#include "common.h"

namespace oh_my_loam {

struct FeaturePoints {
  IPointCloud feature_pts;  // all feature points
  IPointCloud sharp_corner_pts;
  IPointCloud less_sharp_corner_pts;
  IPointCloud flat_surf_pts;
  IPointCloud less_flat_surf_pts;
};

class FeaturePointsExtractor {
 public:
  FeaturePointsExtractor() = default;
  virtual ~FeaturePointsExtractor() = default;
  FeaturePointsExtractor(const FeaturePointsExtractor&) = delete;
  FeaturePointsExtractor& operator=(const FeaturePointsExtractor&) = delete;

  void Extract(const PointCloud& cloud_in, FeaturePoints* const feature) const;

  int num_scans() const { return num_scans_; }

 protected:
  virtual int GetScanID(const Point& pt) const = 0;

  void SplitScan(const PointCloud& cloud,
                 std::vector<IPointCloud>* const scans) const;

  // void ComputePointCurvature(IPointCloud* const scan) const;

  // void AssignPointType(IPointCloud* const scan) const;

  int num_scans_ = 0;
};

}  // namespace oh_my_loam