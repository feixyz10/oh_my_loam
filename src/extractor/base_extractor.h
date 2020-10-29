#pragma once

#include "feature_points.h"
#include "visualizer/extractor_visualizer.h"

namespace oh_my_loam {

class Extractor {
 public:
  Extractor() = default;
  virtual ~Extractor() = default;

  bool Init(const YAML::Node& config);
  void Process(const PointCloud& cloud, FeaturePoints* const feature);

  int num_scans() const { return num_scans_; }

 protected:
  virtual int GetScanID(const Point& pt) const = 0;

  int num_scans_ = 0;

  YAML::Node config_;

 private:
  void Visualize(const PointCloud& cloud, const FeaturePoints& feature_pts,
                 double timestamp = std::nan(""));

  void SplitScan(const PointCloud& cloud,
                 std::vector<TCTPointCloud>* const scans) const;

  void ComputePointCurvature(TCTPointCloud* const scan,
                             bool remove_nan = true) const;

  void AssignPointType(TCTPointCloud* const scan) const;

  void SetNeighborsPicked(const TCTPointCloud& scan, size_t ix,
                          std::vector<bool>* const picked) const;

  void GenerateFeaturePoints(const TCTPointCloud& scan,
                             FeaturePoints* const feature) const;

  bool is_vis_ = false;

  std::unique_ptr<ExtractorVisualizer> visualizer_{nullptr};

  DISALLOW_COPY_AND_ASSIGN(Extractor);
};

}  // namespace oh_my_loam