#pragma once

#include <sys/stat.h>

#include <mutex>
#include <shared_mutex>
#include <vector>

#include "common/geometry/pose3d.h"
#include "oh_my_loam/base/helper.h"
#include "oh_my_loam/base/types.h"
#include "oh_my_loam/mapper/map.h"

namespace oh_my_loam {

class Mapper {
 public:
  Mapper() = default;

  bool Init();

  void Process(double timestamp, const TPointCloudConstPtr &cloud_corn,
               const TPointCloudConstPtr &cloud_surf,
               const common::Pose3d &pose_curr2odom,
               common::Pose3d *const pose_curr2map);

  TPointCloudPtr GetCornMapPoints() const {
    std::shared_lock<std::shared_mutex> lock(corn_map_mutex_);
    return corn_map_->GetAllPoints();
  }

  TPointCloudPtr GetSurfMapPoints() const {
    std::shared_lock<std::shared_mutex> lock(surf_map_mutex_);
    return surf_map_->GetAllPoints();
  }

  TPointCloudPtr GetMapPoints() const {
    TPointCloudPtr map_points(new TPointCloud);
    *map_points += *GetCornMapPoints();
    *map_points += *GetSurfMapPoints();
    return map_points;
  }

  void Reset();

 private:
  enum State { DONE, RUNNING, UN_INIT };

  void Run(const TPointCloudConstPtr &cloud_corn,
           const TPointCloudConstPtr &cloud_surf, const Pose3d &pose_init);

  void MatchCorn(const TPointCloudConstPtr &src, const TPointCloudConstPtr &tgt,
                 std::vector<PointLinePair> *const pair) const;

  void MatchSurf(const TPointCloudConstPtr &src, const TPointCloudConstPtr &tgt,
                 std::vector<PointPlanePair> *const pair) const;

  State GetState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
  }

  void SetState(State state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = state;
  }

  void AdjustMap(const Index &index);

  void Visualize();

  YAML::Node config_;

  mutable std::shared_mutex corn_map_mutex_;
  mutable std::shared_mutex surf_map_mutex_;
  std::unique_ptr<Map> corn_map_;
  std::unique_ptr<Map> surf_map_;

  mutable std::mutex state_mutex_;
  Pose3d pose_odom2map_;
  State state_ = UN_INIT;

  mutable std::unique_ptr<std::thread> thread_{nullptr};

  bool is_vis_ = false;

  bool verbose_ = false;

  DISALLOW_COPY_AND_ASSIGN(Mapper)
};

}  // namespace oh_my_loam