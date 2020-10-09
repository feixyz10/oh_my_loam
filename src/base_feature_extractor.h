#pragma once

#include "common/utils.h"

namespace oh_loam {

struct FeaturePoints {
  IPointCloudPtr laser_cloud;
  IPointCloudPtr sharp_corner_points;
  IPointCloudPtr less_sharp_corner_points;
  IPointCloudPtr flat_surf_points;
  IPointCloudPtr less_flat_surf_points;

  FeaturePoints() {
    laser_cloud.reset(new IPointCloud);
    sharp_corner_points.reset(new IPointCloud);
    less_sharp_corner_points.reset(new IPointCloud);
    flat_surf_points.reset(new IPointCloud);
    less_flat_surf_points.reset(new IPointCloud);
  }
};

class FeaturePointsExtractor {
 public:
  FeaturePointsExtractor() = default;
  virtual ~FeaturePointsExtractor() = default;
  FeaturePointsExtractor(const FeaturePointsExtractor&) = delete;
  FeaturePointsExtractor& operator=(const FeaturePointsExtractor&) = delete;

  bool Extract(const PointCloud& cloud_in, FeaturePoints* const feature) const;

  int num_scans() const { return num_scans_; }

 protected:
  virtual GetScanID(const Point& pt) const = 0;

  void SplitScan(const PointCloud& cloud,
                 std::vector<IPointCloud>* const scans) const;

  void ComputePointCurvature(IPointCloud* const scan) const;

  void AssignPointType(IPointCloud* const scan) const;

  int num_scans_ = 0;
};

}  // oh_loam