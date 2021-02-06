#pragma once

#include "common/visualizer/lidar_visualizer.h"
#include "oh_my_loam/solver/cost_function.h"

namespace oh_my_loam {

struct MapperVisFrame : public common::LidarVisFrame {
  TPointCloudConstPtr cloud_surf;
  TPointCloudConstPtr cloud_corn;
  std::vector<PointLineCoeffPair> pl_pairs;
  std::vector<PointPlaneCoeffPair> pp_pairs;
  common::Pose3d pose_curr2odom;
  common::Pose3d pose_curr2map;
};

class MapperVisualizer : public common::LidarVisualizer {
 public:
  explicit MapperVisualizer(const std::string &name = "MapperVisualizer",
                            size_t max_history_size = 10)
      : common::LidarVisualizer(name, max_history_size) {}

 private:
  void Draw() override;

  void DrawCorn(const common::Pose3d &pose,
                const std::vector<PointLineCoeffPair> &pairs);

  void DrawSurf(const common::Pose3d &pose,
                const std::vector<PointPlaneCoeffPair> &pairs);

  void DrawTrajectory();

  void KeyboardEventCallback(
      const pcl::visualization::KeyboardEvent &event) override;

  bool trans_ = true;

  std::vector<common::Pose3d> poses_;
};

}  // namespace oh_my_loam
