#include "extractor.h"

#include <cmath>

#include "common/pcl/pcl_utils.h"

namespace oh_my_loam {

namespace {
const double kTwoPi = 2 * M_PI;
const double kEps = 1e-6;
}  // namespace

bool Extractor::Init() {
  const auto &config = common::YAMLConfig::Instance()->config();
  config_ = config["extractor_config"];
  is_vis_ = config["vis"].as<bool>() && config_["vis"].as<bool>();
  verbose_ = config_["verbose"].as<bool>();
  AINFO << "Extraction visualizer: " << (is_vis_ ? "ON" : "OFF");
  if (is_vis_) visualizer_.reset(new ExtractorVisualizer);
  return true;
}

void Extractor::Process(double timestamp,
                        const common::PointCloudConstPtr &cloud,
                        std::vector<Feature> *const features) {
  BLOCK_TIMER_START;
  if (cloud->size() < config_["min_point_num"].as<size_t>()) {
    AWARN << "Too few input points: num = " << cloud->size() << " (< "
          << config_["min_point_num"].as<int>() << ")";
    return;
  }
  // split point cloud int scans
  std::vector<TCTPointCloud> scans;
  SplitScan(*cloud, &scans);
  AINFO_IF(verbose_) << "Extractor::SplitScan: " << BLOCK_TIMER_STOP_FMT;
  // compute curvature to each point
  for (auto &scan : scans) {
    ComputeCurvature(&scan);
  }
  AINFO_IF(verbose_) << "Extractor::ComputeCurvature: " << BLOCK_TIMER_STOP_FMT;
  // assign type to each point: FLAT, LESS_FLAT, NORMAL, LESS_SHARP or SHARP
  for (auto &scan : scans) {
    AssignType(&scan);
  }
  AINFO_IF(verbose_) << "Extractor::AssignType: " << BLOCK_TIMER_STOP_FMT;
  // store points into feature point clouds based on their type
  for (size_t i = 0; i < scans.size(); ++i) {
    Feature feature;
    GenerateFeature(scans[i], &feature);
    features->push_back(std::move(feature));
  }
  AINFO << "Extractor::Process: " << BLOCK_TIMER_STOP_FMT;
  if (is_vis_) Visualize(cloud, *features, timestamp);
}

void Extractor::SplitScan(const common::PointCloud &cloud,
                          std::vector<TCTPointCloud> *const scans) const {
  scans->resize(num_scans_);
  auto it = std::min_element(cloud.begin(), cloud.begin() + num_scans(),
                             [](const auto &p1, const auto &p2) {
                               return -std::atan2(p1.y, p1.x) <
                                      -std::atan2(p2.y, p2.x);
                             });
  double yaw_start = -std::atan2(it->y, it->x);
  bool half_passed = false;
  for (const auto &pt : cloud) {
    int scan_id = GetScanID(pt);
    if (scan_id >= num_scans_ || scan_id < 0) continue;
    double yaw = -std::atan2(pt.y, pt.x);
    double yaw_diff = common::NormalizeAngle(yaw - yaw_start);
    if (yaw_diff >= -kEps) {
      if (half_passed) yaw_diff += kTwoPi;
    } else {
      half_passed = true;
      yaw_diff += kTwoPi;
    }
    double time = std::min(yaw_diff / kTwoPi, 1 - kEps) + scan_id;
    scans->at(scan_id).push_back(
        {pt.x, pt.y, pt.z, static_cast<float>(time), std::nanf("")});
  }
}

void Extractor::ComputeCurvature(TCTPointCloud *const scan) const {
  size_t seg_num = config_["scan_seg_num"].as<int>();
  if (scan->size() <= 10 + seg_num) return;
  auto &pts = scan->points;
  for (size_t i = 5; i < pts.size() - 5; ++i) {
    float dx = pts[i - 5].x + pts[i - 4].x + pts[i - 3].x + pts[i - 2].x +
               pts[i - 1].x + pts[i + 1].x + pts[i + 2].x + pts[i + 3].x +
               pts[i + 4].x + pts[i + 5].x - 10 * pts[i].x;
    float dy = pts[i - 5].y + pts[i - 4].y + pts[i - 3].y + pts[i - 2].y +
               pts[i - 1].y + pts[i + 1].y + pts[i + 2].y + pts[i + 3].y +
               pts[i + 4].y + pts[i + 5].y - 10 * pts[i].y;
    float dz = pts[i - 5].z + pts[i - 4].z + pts[i - 3].z + pts[i - 2].z +
               pts[i - 1].z + pts[i + 1].z + pts[i + 2].z + pts[i + 3].z +
               pts[i + 4].z + pts[i + 5].z - 10 * pts[i].z;
    pts[i].curvature = std::hypot(dx, dy, dz);
  }
  common::RemovePoints<TCTPoint>(*scan, scan, [](const TCTPoint &pt) {
    return !std::isfinite(pt.curvature);
  });
}

void Extractor::AssignType(TCTPointCloud *const scan) const {
  int pt_num = scan->size();
  int seg_num = config_["scan_seg_num"].as<int>();
  if (pt_num < seg_num) return;
  int seg_pt_num = (pt_num - 1) / seg_num + 1;
  AERROR << pt_num << " " << seg_num << " " << seg_pt_num;

  std::vector<bool> picked(pt_num, false);
  std::vector<int> indices = common::Range(pt_num);
  int sharp_corner_point_num = config_["sharp_corner_point_num"].as<int>();
  int corner_point_num = config_["corner_point_num"].as<int>();
  int flat_surf_point_num = config_["flat_surf_point_num"].as<int>();
  float corner_point_curvature_th =
      config_["corner_point_curvature_th"].as<float>();
  float surf_point_curvature_th =
      config_["surf_point_curvature_th"].as<float>();
  for (int seg = 0; seg < seg_num; ++seg) {
    int b = seg * seg_pt_num;
    int e = std::min((seg + 1) * seg_pt_num, pt_num);
    // sort by curvature for each segment: large -> small
    std::sort(indices.begin() + b, indices.begin() + e, [&](int i, int j) {
      return scan->at(i).curvature > scan->at(j).curvature;
    });
    // pick corner points
    int corner_point_picked_num = 0;
    for (int i = b; i < e; ++i) {
      size_t ix = indices[i];
      if (!picked.at(ix) &&
          scan->at(ix).curvature > corner_point_curvature_th) {
        ++corner_point_picked_num;
        if (corner_point_picked_num <= sharp_corner_point_num) {
          scan->at(ix).type = PointType::SHARP_CORNER;
        } else if (corner_point_picked_num <= corner_point_num) {
          scan->at(ix).type = PointType::CORNER;
        } else {
          break;
        }
        picked.at(ix) = true;
        UpdateNeighborsPicked(*scan, ix, &picked);
      }
    }
    // pick surface points
    int surf_point_picked_num = 0;
    for (int i = e - 1; i >= b; --i) {
      size_t ix = indices[i];
      if (!picked.at(ix) && scan->at(ix).curvature < surf_point_curvature_th) {
        ++surf_point_picked_num;
        if (surf_point_picked_num <= flat_surf_point_num) {
          scan->at(ix).type = PointType::FLAT_SURF;
        } else {
          break;
        }
        picked.at(ix) = true;
        UpdateNeighborsPicked(*scan, ix, &picked);
      }
    }
  }
}

void Extractor::GenerateFeature(const TCTPointCloud &scan,
                                Feature *const feature) const {
  for (const auto &pt : scan) {
    TPoint point(pt.x, pt.y, pt.z, pt.time);
    switch (pt.type) {
      case PointType::FLAT_SURF:
        feature->cloud_flat_surf->push_back(point);
      // no break: FLAT_SURF points are also SURF points
      case PointType::SURF:
        feature->cloud_surf->push_back(point);
        break;
      case PointType::SHARP_CORNER:
        feature->cloud_sharp_corner->push_back(point);
      // no break: SHARP_CORNER points are also CORNER points
      case PointType::CORNER:
        feature->cloud_corner->push_back(point);
        break;
      default:
        feature->cloud_surf->push_back(point);
        break;
    }
  }
  TPointCloudPtr dowm_sampled(new TPointCloud);
  common::VoxelDownSample<TPoint>(
      feature->cloud_surf, dowm_sampled.get(),
      config_["downsample_voxel_size"].as<double>());
  feature->cloud_surf = dowm_sampled;
}

void Extractor::Visualize(const common::PointCloudConstPtr &cloud,
                          const std::vector<Feature> &features,
                          double timestamp) const {
  std::shared_ptr<ExtractorVisFrame> frame(new ExtractorVisFrame);
  frame->timestamp = timestamp;
  frame->cloud = cloud;
  frame->features = features;
  visualizer_->Render(frame);
}

void Extractor::UpdateNeighborsPicked(const TCTPointCloud &scan, size_t ix,
                                      std::vector<bool> *const picked) const {
  auto dist_sq = [&](size_t i, size_t j) -> double {
    return common::DistanceSquare<TCTPoint>(scan[i], scan[j]);
  };
  double neighbor_point_dist_sq_th =
      config_["neighbor_point_dist_sq_th"].as<float>();
  for (size_t i = 1; i <= 5; ++i) {
    if (ix - i < 0) break;
    if (picked->at(ix - i)) continue;
    if (dist_sq(ix - i, ix - i + 1) > neighbor_point_dist_sq_th) break;
    picked->at(ix - i) = true;
  }
  for (size_t i = 1; i <= 5; ++i) {
    if (ix + i >= scan.size()) break;
    if (picked->at(ix + i)) continue;
    if (dist_sq(ix + i, ix + i - 1) > neighbor_point_dist_sq_th) break;
    picked->at(ix + i) = true;
  }
}

}  // namespace oh_my_loam
