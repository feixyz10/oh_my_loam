#include "odometer_visualizer.h"

#include "common/color/color.h"

namespace oh_my_loam {

namespace {
using namespace common;
}  // namespace

void OdometerVisualizer::Draw() {
  auto frame = GetCurrentFrame<OdometerVisFrame>();
  DrawPointCloud<TPoint>(frame.cloud_corn, GRAY, "cloud_corn");
  DrawPointCloud<TPoint>(frame.cloud_surf, GRAY, "cloud_surf");

  DrawCorn(frame.pose_curr2last, frame.pl_pairs);
  DrawSurf(frame.pose_curr2last, frame.pp_pairs);

  poses_.push_back(frame.pose_curr2world);
  DrawTrajectory();
}

void OdometerVisualizer::DrawCorn(const Pose3d &pose,
                                  const std::vector<PointLinePair> &pairs) {
  TPointCloudPtr src(new TPointCloud);
  TPointCloudPtr tgt(new TPointCloud);
  src->resize(pairs.size());
  tgt->resize(pairs.size() * 2);
  for (size_t i = 0; i < pairs.size(); ++i) {
    const auto &pair = pairs[i];
    src->at(i) = pair.pt;
    if (trans_) TransformToStart(pose, pair.pt, &src->points[i]);
    tgt->at(2 * i) = pair.line.pt1;
    tgt->at(2 * i + 1) = pair.line.pt2;
  }
  DrawPointCloud<TPoint>(tgt, YELLOW, "tgt_corn", 4);
  DrawPointCloud<TPoint>(src, RED, "src_corn", 4);
  if (!corn_connect_) return;
  for (size_t i = 0; i < src->size(); ++i) {
    const auto &p = src->at(i), &p1 = tgt->at(2 * i), &p2 = tgt->at(2 * i + 1);
    common::AddLine<TPoint>(p, p1, WHITE, "corn_line1_" + std::to_string(i),
                            viewer_.get());
    common::AddLine<TPoint>(p, p2, WHITE, "corn_line2_" + std::to_string(i),
                            viewer_.get());
  }
}

void OdometerVisualizer::DrawSurf(const Pose3d &pose,
                                  const std::vector<PointPlanePair> &pairs) {
  TPointCloudPtr src(new TPointCloud);
  TPointCloudPtr tgt(new TPointCloud);
  src->resize(pairs.size());
  tgt->resize(pairs.size() * 3);
  for (size_t i = 0; i < pairs.size(); ++i) {
    const auto &pair = pairs[i];
    src->at(i) = pair.pt;
    if (trans_) TransformToStart(pose, pair.pt, &src->points[i]);
    tgt->at(3 * i) = pair.plane.pt1;
    tgt->at(3 * i + 1) = pair.plane.pt2;
    tgt->at(3 * i + 2) = pair.plane.pt3;
  }
  DrawPointCloud<TPoint>(tgt, BLUE, "tgt_surf", 4);
  DrawPointCloud<TPoint>(src, CYAN, "src_surf", 4);
  if (!surf_connect_) return;
  for (size_t i = 0; i < src->size(); ++i) {
    const auto &p = src->at(i), &p1 = tgt->at(3 * i), &p2 = tgt->at(3 * i + 1),
               &p3 = tgt->at(3 * i + 2);
    AddLine<TPoint>(p, p1, WHITE, "surf_line1_" + std::to_string(i),
                    viewer_.get());
    AddLine<TPoint>(p, p2, WHITE, "surf_line2_" + std::to_string(i),
                    viewer_.get());
    AddLine<TPoint>(p, p3, WHITE, "surf_line3_" + std::to_string(i),
                    viewer_.get());
  }
}

void OdometerVisualizer::DrawTrajectory() {
  std::vector<Pose3d> poses_n;
  poses_n.reserve((poses_.size()));
  Pose3d pose_inv = poses_.back().Inv();
  for (const auto &pose : poses_) {
    poses_n.emplace_back(pose_inv * pose);
  };
  for (size_t i = 0; i < poses_n.size() - 1; ++i) {
    Eigen::Vector3f p1 = poses_n[i].t_vec().cast<float>();
    Eigen::Vector3f p2 = poses_n[i + 1].t_vec().cast<float>();
    AddLine<Point>({p1.x(), p1.y(), p1.z()}, {p2.x(), p2.y(), p2.z()}, PINK,
                   "trajectory" + std::to_string(i), viewer_.get());
  }
}

void OdometerVisualizer::KeyboardEventCallback(
    const pcl::visualization::KeyboardEvent &event) {
  if (event.getKeySym() == "p" && event.keyDown()) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++curr_frame_iter_;
    if (curr_frame_iter_ == history_frames_.end()) {
      --curr_frame_iter_;
    }
    is_updated_ = true;
  } else if (event.getKeySym() == "n" && event.keyDown()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (curr_frame_iter_ != history_frames_.begin()) {
      --curr_frame_iter_;
    }
    is_updated_ = true;
  } else if (event.getKeySym() == "t" && event.keyDown()) {
    trans_ = !trans_;
    is_updated_ = true;
  } else if (event.getKeySym() == "c" && event.keyDown()) {
    corn_connect_ = !corn_connect_;
    is_updated_ = true;
  } else if (event.getKeySym() == "s" && event.keyDown()) {
    surf_connect_ = !surf_connect_;
    is_updated_ = true;
  } else if (event.getKeySym() == "r" && event.keyDown()) {
    viewer_->setCameraPosition(0, 0, 200, 0, 0, 0, 1, 0, 0, 0);
    viewer_->setSize(2500, 1500);
    trans_ = true;
    corn_connect_ = surf_connect_ = false;
    is_updated_ = true;
  }
}

}  // namespace oh_my_loam
