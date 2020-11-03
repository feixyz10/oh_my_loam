#include "odometry_visualizer.h"

namespace oh_my_loam {

void OdometryVisualizer::Draw() {
  auto frame = GetCurrentFrame<OdometryVisFrame>();
  TPointCloud src_corn_pts, tgt_corn_pts;
  src_corn_pts.resize(frame.pl_pairs.size());
  tgt_corn_pts.resize(frame.pl_pairs.size() * 2);
  for (size_t i = 0; i < frame.pl_pairs.size(); ++i) {
    const auto& pair = frame.pl_pairs[i];
    TransformToStart(frame.pose_curr2last, pair.pt, &src_corn_pts[i]);
    tgt_corn_pts[2 * i] = pair.line.pt1;
    tgt_corn_pts[2 * i + 1] = pair.line.pt2;
  }
  TPointCloud src_surf_pts, tgt_surf_pts;
  src_surf_pts.resize(frame.pp_pairs.size());
  tgt_surf_pts.resize(frame.pp_pairs.size() * 3);
  for (size_t i = 0; i < frame.pp_pairs.size(); ++i) {
    const auto& pair = frame.pp_pairs[i];
    TransformToStart(frame.pose_curr2last, pair.pt, &src_surf_pts[i]);
    tgt_surf_pts[3 * i] = pair.plane.pt1;
    tgt_surf_pts[3 * i + 1] = pair.plane.pt2;
    tgt_surf_pts[3 * i + 2] = pair.plane.pt3;
  }
  {  // add src_corn_pts
    std::string id = "src_corn_pts";
    DrawPointCloud(src_corn_pts, CYAN, id, viewer_.get(), 7);
    rendered_cloud_ids_.push_back(id);
  }
  {  // add tgt_corn_pts
    std::string id = "tgt_corn_pts";
    DrawPointCloud(tgt_corn_pts, BLUE, id, viewer_.get(), 4);
    rendered_cloud_ids_.push_back(id);
  }
  {  // add src_surf_pts
    std::string id = "src_surf_pts";
    DrawPointCloud(src_surf_pts, PURPLE, id, viewer_.get(), 7);
    rendered_cloud_ids_.push_back(id);
  }
  {  // add tgt_surf_pts
    std::string id = "tgt_surf_pts";
    DrawPointCloud(tgt_surf_pts, RED, id, viewer_.get(), 4);
    rendered_cloud_ids_.push_back(id);
  }
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
      DrawLine(Point(p1.x(), p1.y(), p1.z()), Point(p2.x(), p2.y(), p2.z()),
               WHITE, "line" + std::to_string(i), viewer_.get());
    } else {
      DrawLine(Point(p1.x(), p1.y(), p1.z()), Point(0, 0, 0), WHITE,
               "line" + std::to_string(i), viewer_.get());
    }
  }
  poses_.push_back(frame.pose_curr2world);
}

}  // namespace oh_my_loam
