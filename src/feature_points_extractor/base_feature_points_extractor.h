#pragma once

#include "feature_points.h"
#include "visualizer/feature_points_visualizer.h"

namespace oh_my_loam {

class FeaturePointsExtractor {
 public:
  FeaturePointsExtractor() = default;
  virtual ~FeaturePointsExtractor() = default;

  bool Init(const YAML::Node& config);
  void Extract(const PointCloud& cloud_in, FeaturePoints* const feature);

  int num_scans() const { return num_scans_; }

 protected:
  virtual int GetScanID(const Point& pt) const = 0;

  void Visualize(const PointCloud& cloud, const FeaturePoints& feature_pts,
                 double timestamp = std::nan(""));

  void SplitScan(const PointCloud& cloud,
                 std::vector<IPointCloud>* const scans) const;

  void ComputePointCurvature(IPointCloud* const scan) const;

  void AssignPointType(IPointCloud* const scan) const;

  int num_scans_ = 0;

  bool is_vis_ = false;

  std::unique_ptr<FeaturePointsVisualizer> visualizer_{nullptr};

  YAML::Node config_;

  DISALLOW_COPY_AND_ASSIGN(FeaturePointsExtractor);
};

}  // namespace oh_my_loam