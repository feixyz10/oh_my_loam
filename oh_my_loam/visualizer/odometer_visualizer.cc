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

  traj_.AddPose(frame.pose_curr2world);
  AddTrajectory(traj_.Copy(true), PINK, "trajectory", viewer_.get());
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
  } else if (event.getKeySym() == "r" && event.keyDown()) {
    viewer_->setCameraPosition(0, 0, 200, 0, 0, 0, 1, 0, 0, 0);
    viewer_->setSize(2500, 1500);
    trans_ = true;
    is_updated_ = true;
  }
}

}  // namespace oh_my_loam
