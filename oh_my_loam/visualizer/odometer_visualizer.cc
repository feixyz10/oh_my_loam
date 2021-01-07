#include "odometer_visualizer.h"

namespace oh_my_loam {

namespace {
using namespace common;
}  // namespace

void OdometerVisualizer::Draw() {
  auto frame = GetCurrentFrame<OdometerVisFrame>();
  TPointCloudPtr src_corn_pts(new TPointCloud);
  TPointCloudPtr tgt_corn_pts(new TPointCloud);
  src_corn_pts->resize(frame.pl_pairs.size());
  tgt_corn_pts->resize(frame.pl_pairs.size() * 2);
  for (size_t i = 0; i < frame.pl_pairs.size(); ++i) {
    const auto& pair = frame.pl_pairs[i];
    TransformToStart(frame.pose_curr2last, pair.pt, &src_corn_pts->points[i]);
    tgt_corn_pts->points[2 * i] = pair.line.pt1;
    tgt_corn_pts->points[2 * i + 1] = pair.line.pt2;
  }
  TPointCloudPtr src_surf_pts(new TPointCloud);
  TPointCloudPtr tgt_surf_pts(new TPointCloud);
  src_surf_pts->resize(frame.pp_pairs.size());
  tgt_surf_pts->resize(frame.pp_pairs.size() * 3);
  for (size_t i = 0; i < frame.pp_pairs.size(); ++i) {
    const auto& pair = frame.pp_pairs[i];
    TransformToStart(frame.pose_curr2last, pair.pt, &src_surf_pts->points[i]);
    tgt_surf_pts->points[3 * i] = pair.plane.pt1;
    tgt_surf_pts->points[3 * i + 1] = pair.plane.pt2;
    tgt_surf_pts->points[3 * i + 2] = pair.plane.pt3;
  }
  DrawPointCloud<TPoint>(src_corn_pts, CYAN, "src_corn_pts", 7);
  DrawPointCloud<TPoint>(tgt_corn_pts, BLUE, "tgt_corn_pts", 4);
  DrawPointCloud<TPoint>(src_surf_pts, PURPLE, "src_surf_pts", 7);
  DrawPointCloud<TPoint>(tgt_surf_pts, RED, "tgt_surf_pts", 4);
  std::vector<Pose3D> poses_n;
  poses_n.reserve((poses_.size()));
  Pose3D pose_inv = frame.pose_curr2world.Inv();
  for (const auto& pose : poses_) {
    poses_n.emplace_back(pose_inv * pose);
  };
  for (size_t i = 0; i < poses_n.size(); ++i) {
    Eigen::Vector3f p1 = poses_n[i].p().cast<float>();
    if (i < poses_n.size() - 1) {
      Eigen::Vector3f p2 = poses_n[i + 1].p().cast<float>();
      AddLine(Point(p1.x(), p1.y(), p1.z()), Point(p2.x(), p2.y(), p2.z()),
              WHITE, "line" + std::to_string(i), viewer_.get());
    } else {
      AddLine(Point(p1.x(), p1.y(), p1.z()), Point(0, 0, 0), WHITE,
              "line" + std::to_string(i), viewer_.get());
    }
  }
  poses_.push_back(frame.pose_curr2world);
}

}  // namespace oh_my_loam
