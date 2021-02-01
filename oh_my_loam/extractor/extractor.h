#pragma once

#include "common/common.h"
#include "oh_my_loam/base/feature.h"
#include "oh_my_loam/base/utils.h"
#include "oh_my_loam/base/types.h"
#include "oh_my_loam/visualizer/extractor_visualizer.h"

namespace oh_my_loam {

class Extractor {
 public:
  Extractor() = default;
  virtual ~Extractor() = default;

  bool Init();

  void Process(double timestamp, const common::PointCloudConstPtr &cloud,
               std::vector<Feature> *const features);

  int num_scans() const {
    return num_scans_;
  }

  virtual void Reset() {}

 protected:
  virtual int GetScanID(const common::Point &pt) const = 0;

  virtual void SplitScan(const common::PointCloud &cloud,
                         std::vector<TCTPointCloud> *const scans) const;

  virtual void ComputeCurvature(TCTPointCloud *const scan) const;

  virtual void AssignType(TCTPointCloud *const scan) const;

  virtual void GenerateFeature(const TCTPointCloud &scan,
                               Feature *const feature) const;

  virtual void Visualize(const common::PointCloudConstPtr &cloud,
                         const std::vector<Feature> &features,
                         double timestamp = 0.0) const;

  int num_scans_ = 0;

  YAML::Node config_;

  std::unique_ptr<ExtractorVisualizer> visualizer_{nullptr};

  bool verbose_ = false;

 private:
  void UpdateNeighborsPicked(const TCTPointCloud &scan, int ix,
                             std::vector<bool> *const picked) const;

  bool is_vis_ = false;

  DISALLOW_COPY_AND_ASSIGN(Extractor);
};

}  // namespace oh_my_loam