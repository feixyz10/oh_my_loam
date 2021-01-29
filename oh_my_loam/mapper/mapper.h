#pragma once

#include <sys/stat.h>

#include <mutex>
#include <vector>

#include "common/geometry/pose3d.h"
#include "oh_my_loam/mapper/map.h"

namespace oh_my_loam {

class Mapper {
 public:
  Mapper() = default;

  bool Init();

  void Process(double timestamp, const TPointCloudConstPtr &cloud_corn,
               const TPointCloudConstPtr &cloud_surf,
               common::Pose3d *const pose_out);

  void Reset();

 private:
  enum State { DONE, RUNNING, UN_INIT };

  void Run(double timestamp, const TPointCloudConstPtr &cloud_corn,
           const TPointCloudConstPtr &cloud_surf);

  void TransformUpdate();

  void MapUpdate();

  State GetState() {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  void SetState(State state) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
  }

  void Visualize();

  YAML::Node config_;

  struct TimePose {
    double timestamp;
    common::Pose3d pose;
  };

  std::unique_ptr<Map> cloud_corn_map_;
  std::unique_ptr<Map> cloud_surf_map_;

  std::mutex mutex_;
  std::vector<TimePose> poses_;
  std::unique_ptr<std::thread> thread_{nullptr};

  State state_ = UN_INIT;

  bool is_vis_ = false;

  bool verbose_ = false;

  DISALLOW_COPY_AND_ASSIGN(Mapper)
};

}  // namespace oh_my_loam