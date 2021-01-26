#include "ohmyloam_visualizer.h"

#include "common/color/color.h"

namespace oh_my_loam {

namespace {
using namespace common;
#define LIGHT_BLUE common::Color(0, 0, 80)
#define LIGHT_YELLOW common::Color(80, 80, 0)
}  // namespace

void OhmyloamVisualizer::Draw() {
  auto frame = GetCurrentFrame<OhmyloamVisFrame>();
  DrawPointCloud<TPoint>(frame.cloud_map_corn, LIGHT_YELLOW, "cloud_map_corn");
  DrawPointCloud<TPoint>(frame.cloud_map_surf, LIGHT_BLUE, "cloud_map_surf");

  DrawPointCloud<TPoint>(frame.cloud_corn, RED, "cloud_corn");
  DrawPointCloud<TPoint>(frame.cloud_surf, CYAN, "cloud_surf");

  DrawTrajectory(frame.poses);
}

void OhmyloamVisualizer::DrawTrajectory(std::vector<common::Pose3d> &poses) {
  std::vector<Pose3d> poses_n;
  poses_n.reserve((poses.size()));
  Pose3d pose_inv = poses.back().Inv();
  for (const auto &pose : poses) {
    poses_n.emplace_back(pose_inv * pose);
  };
  for (size_t i = 0; i < poses_n.size() - 1; ++i) {
    Eigen::Vector3f p1 = poses_n[i].t_vec().cast<float>();
    Eigen::Vector3f p2 = poses_n[i + 1].t_vec().cast<float>();
    AddLine<Point>({p1.x(), p1.y(), p1.z()}, {p2.x(), p2.y(), p2.z()}, PINK,
                   "trajectory" + std::to_string(i), viewer_.get());
  }
}

}  // namespace oh_my_loam