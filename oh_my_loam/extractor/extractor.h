#pragma once

#include "base/feature.h"
#include "base/types.h"
#include "common/common.h"
#include "visualizer/extractor_visualizer.h"

namespace oh_my_loam {

class Extractor {
 public:
  Extractor() = default;
  virtual ~Extractor() = default;

  bool Init();

  void Process(const common::PointCloudConstPtr& cloud, Feature* const feature);

  int num_scans() const { return num_scans_; }

 protected:
  virtual int GetScanID(const Point& pt) const = 0;

  YAML::Node config_;

  virtual void SplitScan(const common::PointCloud& cloud,
                         std::vector<TCTPointCloud>* const scans) const;

  virtual void ComputeCurvature(TCTPointCloud* const scan) const;

  virtual void AssignType(TCTPointCloud* const scan) const;

  virtual void GenerateFeature(const TCTPointCloud& scan,
                               Feature* const feature) const;

  int num_scans_ = 0;

  std::unique_ptr<ExtractorVisualizer> visualizer_{nullptr};

  bool verbose_ = false;

 private:
  void Visualize(const common::PointCloudConstPtr& cloud,
                 const Feature& feature, double timestamp = 0.0);

  void SetNeighborsPicked(const TCTPointCloud& scan, size_t ix,
                          std::vector<bool>* const picked) const;

  bool is_vis_ = false;

  DISALLOW_COPY_AND_ASSIGN(Extractor);
};

}  // namespace oh_my_loam