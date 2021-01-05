#include "extractor.h"

#include <cmath>

#include "common/pcl/pcl_utils.h"

namespace oh_my_loam {

namespace {
const int kScanSegNum = 6;
const double kTwoPi = 2 * M_PI;
using namespace common;
}  // namespace

bool Extractor::Init() {
  const auto& config = YAMLConfig::Instance()->config();
  config_ = config["extractor_config"];
  is_vis_ = config["vis"].as<bool>() && config_["vis"].as<bool>();
  verbose_ = config_["verbose"].as<bool>();
  AINFO << "Extraction visualizer: " << (is_vis_ ? "ON" : "OFF");
  if (is_vis_) visualizer_.reset(new ExtractorVisualizer);
  return true;
}

void Extractor::Process(const PointCloudConstPtr& cloud,
                        Feature* const feature) {
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
  // compute curvature for each point in each scan
  for (auto& scan : scans) {
    ComputeCurvature(&scan);
  }
  AINFO_IF(verbose_) << "Extractor::ComputeCurvature: " << BLOCK_TIMER_STOP_FMT;
  // assign type to each point in each scan: FLAT, LESS_FLAT,
  // NORMAL, LESS_SHARP or SHARP
  for (auto& scan : scans) {
    AssignType(&scan);
  }
  AINFO_IF(verbose_) << "Extractor::AssignType: " << BLOCK_TIMER_STOP_FMT;
  // store points into feature point clouds according to their type
  for (const auto& scan : scans) {
    GenerateFeature(scan, feature);
  }
  AINFO << "Extractor::Process: " << BLOCK_TIMER_STOP_FMT;
  if (is_vis_) Visualize(*cloud, *feature);
}

void Extractor::SplitScan(const PointCloud& cloud,
                          std::vector<TCTPointCloud>* const scans) const {
  scans->resize(num_scans_);
  double yaw_start = -atan2(cloud.points[0].y, cloud.points[0].x);
  bool half_passed = false;
  for (const auto& pt : cloud.points) {
    int scan_id = GetScanID(pt);
    if (scan_id >= num_scans_ || scan_id < 0) continue;
    double yaw = -atan2(pt.y, pt.x);
    double yaw_diff = NormalizeAngle(yaw - yaw_start);
    if (yaw_diff > 0) {
      if (half_passed) yaw_start += kTwoPi;
    } else {
      half_passed = true;
      yaw_start += kTwoPi;
    }
    (*scans)[scan_id].points.emplace_back(
        pt.x, pt.y, pt.z, yaw_diff / kTwoPi + scan_id, std::nanf(""));
  }
}

void Extractor::ComputeCurvature(TCTPointCloud* const scan) const {
  if (scan->size() < 20) return;
  auto& pts = scan->points;
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
    pts[i].curvature = std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  RemovePoints<TCTPoint>(*scan, scan, [](const TCTPoint& pt) {
    return !std::isfinite(pt.curvature);
  });
}

void Extractor::AssignType(TCTPointCloud* const scan) const {
  int pt_num = scan->size();
  ACHECK(pt_num >= kScanSegNum);
  int seg_pt_num = (pt_num - 1) / kScanSegNum + 1;
  std::vector<bool> picked(pt_num, false);
  std::vector<int> indices = Range(pt_num);
  int sharp_corner_point_num = config_["sharp_corner_point_num"].as<int>();
  int corner_point_num = config_["corner_point_num"].as<int>();
  int flat_surf_point_num = config_["flat_surf_point_num"].as<int>();
  int surf_point_num = config_["surf_point_num"].as<int>();
  float corner_point_curvature_thres =
      config_["corner_point_curvature_thres"].as<float>();
  float surf_point_curvature_thres =
      config_["surf_point_curvature_thres"].as<float>();
  for (int seg = 0; seg < kScanSegNum; ++seg) {
    int s = seg * seg_pt_num;
    int e = std::min((seg + 1) * seg_pt_num, pt_num);
    // sort by curvature for each segment: large -> small
    std::sort(indices.begin() + s, indices.begin() + e, [&](int i, int j) {
      return scan->at(i).curvature > scan->at(j).curvature;
    });
    // pick corner points
    int corner_pt_picked_num = 0;
    for (int i = s; i < e; ++i) {
      size_t ix = indices[i];
      if (!picked.at(ix) &&
          scan->at(ix).curvature > corner_point_curvature_thres) {
        ++corner_pt_picked_num;
        if (corner_pt_picked_num <= sharp_corner_point_num) {
          scan->at(ix).type = Type::SHARP;
        } else if (corner_pt_picked_num <= corner_point_num) {
          scan->at(ix).type = Type::LESS_SHARP;
        } else {
          break;
        }
        picked.at(ix) = true;
        SetNeighborsPicked(*scan, ix, &picked);
      }
    }
    // pick surface points
    int surf_pt_picked_num = 0;
    for (int i = e - 1; i >= s; --i) {
      size_t ix = indices[i];
      if (!picked.at(ix) &&
          scan->at(ix).curvature < surf_point_curvature_thres) {
        ++surf_pt_picked_num;
        if (surf_pt_picked_num <= flat_surf_point_num) {
          scan->at(ix).type = Type::FLAT;
        } else if (surf_pt_picked_num <= surf_point_num) {
          scan->at(ix).type = Type::LESS_FLAT;
        } else {
          break;
        }
        picked.at(ix) = true;
        SetNeighborsPicked(*scan, ix, &picked);
      }
    }
  }
}

void Extractor::GenerateFeature(const TCTPointCloud& scan,
                                Feature* const feature) const {
  TPointCloudPtr cloud_less_flat_surf(new TPointCloud);
  for (const auto& pt : scan.points) {
    TPoint point(pt.x, pt.y, pt.z, pt.time);
    switch (pt.type) {
      case Type::FLAT:
        feature->cloud_flat_surf->points.emplace_back(point);
      // no break: FLAT points are also LESS_FLAT
      case Type::LESS_FLAT:
        cloud_less_flat_surf->points.emplace_back(point);
        break;
      case Type::SHARP:
        feature->cloud_sharp_corner->points.emplace_back(point);
      // no break: SHARP points are also LESS_SHARP
      case Type::LESS_SHARP:
        feature->cloud_less_sharp_corner->points.emplace_back(point);
        break;
      default:
        // all the rest are also LESS_FLAT
        cloud_less_flat_surf->points.emplace_back(pt.x, pt.y, pt.z, pt.time);
        break;
    }
  }
  TPointCloudPtr dowm_sampled(new TPointCloud);
  VoxelDownSample<TPoint>(cloud_less_flat_surf, dowm_sampled.get(),
                          config_["downsample_voxel_size"].as<double>());
  feature->cloud_less_flat_surf = dowm_sampled;
}

void Extractor::Visualize(const PointCloud& cloud, const Feature& feature,
                          double timestamp) {
  std::shared_ptr<ExtractorVisFrame> frame(new ExtractorVisFrame);
  frame->timestamp = timestamp;
  frame->cloud = cloud.makeShared();
  frame->feature = feature;
  visualizer_->Render(frame);
}

void Extractor::SetNeighborsPicked(const TCTPointCloud& scan, size_t ix,
                                   std::vector<bool>* const picked) const {
  auto DistSqure = [&](int i, int j) -> float {
    float dx = scan.at(i).x - scan.at(j).x;
    float dy = scan.at(i).y - scan.at(j).y;
    float dz = scan.at(i).z - scan.at(j).z;
    return dx * dx + dy * dy + dz * dz;
  };
  float neighbor_point_dist_thres =
      config_["neighbor_point_dist_thres"].as<float>();
  for (size_t i = 1; i <= 5; ++i) {
    if (ix < i) break;
    if (picked->at(ix - i)) continue;
    if (DistSqure(ix - i, ix - i + 1) > neighbor_point_dist_thres) break;
    picked->at(ix - i) = true;
  }
  for (size_t i = 1; i <= 5; ++i) {
    if (ix + i >= scan.size()) break;
    if (picked->at(ix + i)) continue;
    if (DistSqure(ix + i, ix + i - 1) > neighbor_point_dist_thres) break;
    picked->at(ix + i) = true;
  }
}

}  // namespace oh_my_loam
