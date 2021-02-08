#pragma once

#include <memory>
#include "common/geometry/pose3d.h"

namespace common {

class Trajectory {
 public:
  typedef std::shared_ptr<Trajectory> Ptr;
  typedef std::shared_ptr<const Trajectory> ConstPtr;

  Trajectory() = default;

  explicit Trajectory(const std::vector<Pose3d> &poses) {
    poses_ = poses;
  }

  size_t size() const {
    return poses_.size();
  }

  bool empty() const {
    return poses_.empty();
  }

  const Pose3d &at(size_t i) const {
    return poses_.at(i);
  }

  void AddPose(const Pose3d &pose) {
    poses_.push_back(pose);
  }

  Trajectory Copy(bool last_as_origin = false) const {
    Trajectory traj;
    if (empty()) return traj;
    if (!last_as_origin) return *this;
    const Pose3d &last_inv = poses_.back().Inv();
    for (const auto &pose : poses_) {
      traj.AddPose(last_inv * pose);
    }
    return traj;
  }

  std::vector<Eigen::Vector3d> GetPointSeq(bool last_as_origin = false) const {
    std::vector<Eigen::Vector3d> point_seq;
    if (empty()) return point_seq;
    const Pose3d &last_inv = poses_.back().Inv();
    for (const auto &pose : poses_) {
      if (last_as_origin) {
        point_seq.push_back((last_inv * pose).t_vec());
      } else {
        point_seq.push_back(pose.t_vec());
      }
    }
    return point_seq;
  };

 private:
  std::vector<Pose3d> poses_;
};

}  // namespace common