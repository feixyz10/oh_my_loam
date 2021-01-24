#include "odometer_visualizer.h"

#include "common/color/color.h"

namespace oh_my_loam {

namespace {
using namespace common;
}  // namespace

void OdometerVisualizer::Draw() {
  auto frame = GetCurrentFrame<OdometerVisFrame>();
  TPointCloudPtr src_corn(new TPointCloud);
  TPointCloudPtr tgt_corn(new TPointCloud);
  src_corn->resize(frame.pl_pairs.size());
  tgt_corn->resize(frame.pl_pairs.size() * 2);
  for (size_t i = 0; i < frame.pl_pairs.size(); ++i) {
    const auto &pair = frame.pl_pairs[i];
    TransformToStart(frame.pose_curr2last, pair.pt, &src_corn->points[i]);
    tgt_corn->points[2 * i] = pair.line.pt1;
    tgt_corn->points[2 * i + 1] = pair.line.pt2;
  }
  TPointCloudPtr src_surf(new TPointCloud);
  TPointCloudPtr tgt_surf(new TPointCloud);
  src_surf->resize(frame.pp_pairs.size());
  tgt_surf->resize(frame.pp_pairs.size() * 3);
  for (size_t i = 0; i < frame.pp_pairs.size(); ++i) {
    const auto &pair = frame.pp_pairs[i];
    TransformToStart(frame.pose_curr2last, pair.pt, &src_surf->points[i]);
    tgt_surf->points[3 * i] = pair.plane.pt1;
    tgt_surf->points[3 * i + 1] = pair.plane.pt2;
    tgt_surf->points[3 * i + 2] = pair.plane.pt3;
  }
  DrawPointCloud<TPoint>(tgt_corn, YELLOW, "tgt_corn", 4);
  DrawPointCloud<TPoint>(src_corn, RED, "src_corn", 4);
  DrawPointCloud<TPoint>(tgt_surf, BLUE, "tgt_surf", 4);
  DrawPointCloud<TPoint>(src_surf, CYAN, "src_surf", 4);
  std::vector<Pose3d> poses_n;
  poses_n.reserve((poses_.size()));
  Pose3d pose_inv = frame.pose_curr2world.Inv();
  for (const auto &pose : poses_) {
    poses_n.emplace_back(pose_inv * pose);
  };
  for (size_t i = 0; i < poses_n.size(); ++i) {
    Eigen::Vector3f p1 = poses_n[i].t_vec().cast<float>();
    if (i < poses_n.size() - 1) {
      Eigen::Vector3f p2 = poses_n[i + 1].t_vec().cast<float>();
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
