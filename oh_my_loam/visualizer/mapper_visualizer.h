#pragma once

#include "common/geometry/trajectory.h"
#include "common/visualizer/lidar_visualizer.h"
#include "oh_my_loam/solver/cost_function.h"

namespace oh_my_loam {

struct MapperVisFrame : public common::LidarVisFrame {
  TPointCloudConstPtr cloud_corn;
  TPointCloudConstPtr cloud_surf;
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

  void DrawCorn(const common::Pose3d &pose_odom, const common::Pose3d &pose_map,
                const std::vector<PointLineCoeffPair> &pairs);

  void DrawSurf(const common::Pose3d &pose_odom, const common::Pose3d &pose_map,
                const std::vector<PointPlaneCoeffPair> &pairs);

  void DrawTrajectory();

  void KeyboardEventCallback(
      const pcl::visualization::KeyboardEvent &event) override;

  bool trans_ = true;

  common::Trajectory traj_odom_;
  common::Trajectory traj_map_;
};

}  // namespace oh_my_loam
