#include "oh_my_loam/mapper/map.h"

#include "common/pcl/pcl_utils.h"

namespace oh_my_loam {

Map::Map(const std::vector<int> &shape, const std::vector<double> &step) {
  ACHECK(shape.size() == 3);
  std::copy_n(shape.begin(), 3, shape_);
  ACHECK(shape_[0] % 2 == 1 && shape_[1] % 2 == 1 && shape_[2] % 2 == 1);
  std::copy_n(step.begin(), 3, step_);
  ACHECK(step_[0] > 0.0 && step_[1] > 0.0 && step_[2] > 0.0);
  center_[0] = shape_[0] / 2;
  center_[1] = shape_[1] / 2;
  center_[2] = shape_[2] / 2;
  for (int i = 0; i < shape_[0]; ++i) map_.emplace_back(shape_[1], shape_[2]);
}

Map::Map(const std::vector<int> &shape, double step)
    : Map(shape, {step, step, step}) {}

void Map::clear() {
  for (auto &grid : map_) grid.clear();
}

TPointCloudPtr &Map::at(int z_idx, int y_idx, int x_idx) {
  return map_.at(z_idx).at(y_idx).at(x_idx);
}

const TPointCloudPtr &Map::at(int z_idx, int y_idx, int x_idx) const {
  return map_.at(z_idx).at(y_idx).at(x_idx);
}

TPointCloudPtr &Map::at(const Index &index) {
  return map_.at(index.k).at(index.j).at(index.i);
}

const TPointCloudPtr &Map::at(const Index &index) const {
  return map_.at(index.k).at(index.j).at(index.i);
}

void Map::ShiftZ(int n) {
  if (n == 0) return;
  if (n < 0) {
    for (int k = shape_[0] - 1; k >= -n; --k) {
      std::swap(this->at(k), this->at(k + n));
    }
    for (int k = 0; k < -n; ++k) this->at(k).clear();
  } else {
    for (int k = 0; k < shape_[0] - n; ++k) {
      std::swap(this->at(k), this->at(k + n));
    }
    for (int k = shape_[0] - n; k < shape_[0]; ++k) {
      this->at(k).clear();
    }
  }
  center_[0] -= n;
}

void Map::ShiftY(int n) {
  if (n == 0) return;
  for (int k = 0; k < shape_[0]; ++k) {
    if (n < 0) {
      for (int j = shape_[1] - 1; j >= -n; --j) {
        std::swap(this->at(k, j), this->at(k, j + n));
      }
      for (int j = 0; j < -n; ++j) this->at(k, j).clear();
    } else {
      for (int j = 0; j < shape_[1] - n; ++j) {
        std::swap(this->at(k, j), this->at(k, j + n));
      }
      for (int j = shape_[1] - n; j < shape_[1]; ++j) {
        this->at(k, j).clear();
      }
    }
  }
  center_[1] -= n;
}

void Map::ShiftX(int n) {
  if (n == 0) return;
  for (int k = 0; k < shape_[0]; ++k) {
    for (int j = 0; j < shape_[1]; ++j) {
      if (n < 0) {
        for (int i = shape_[2] - 1; i >= -n; --i) {
          std::swap(this->at(k, j, i), this->at(k, j, i + n));
        }
        for (int i = 0; i < -n; ++i) this->at(k, j, i)->clear();

      } else {
        for (int i = 0; i < shape_[2] - n; ++i) {
          std::swap(this->at(k, j, i), this->at(k, j, i + n));
        }
        for (int i = shape_[2] - n; i < shape_[2]; ++i) {
          this->at(k, j, i)->clear();
        }
      }
    }
  }
  center_[2] -= n;
}

Index Map::GetIndex(const TPoint &point) const {
  Index index;
  index.k = static_cast<int>(point.z / step_[0]) + center_[0];
  index.j = static_cast<int>(point.y / step_[1]) + center_[1];
  index.i = static_cast<int>(point.x / step_[2]) + center_[2];
  return index;
}

bool Map::CheckIndex(const Index &index) const {
  int k = index.k, j = index.j, i = index.i;
  return k >= 0 && k < shape_[0] && j >= 0 && j < shape_[1] && i >= 0 &&
         i < shape_[2];
}

TPointCloudPtr Map::GetSubmapPoints(
    const TPoint &point, const std::vector<int> &submap_shapes) const {
  TPointCloudPtr cloud(new TPointCloud);
  for (const auto &index : GetSubmapIndices(point, submap_shapes)) {
    *cloud += *this->at(index);
  }
  return cloud;
}

TPointCloudPtr Map::GetAllPoints() const {
  TPointCloudPtr cloud_all(new TPointCloud);
  for (const auto &grid : map_) *cloud_all += *grid.GetAllPoints();
  return cloud_all;
}

void Map::AddPoints(const TPointCloudConstPtr &cloud,
                    std::vector<Index> *const indices) {
  std::set<Index, Index::Comp> index_set;
  for (const auto &point : *cloud) {
    Index index = GetIndex(point);
    if (!CheckIndex(index)) continue;
    this->at(index)->push_back(point);
    if (indices) index_set.insert(index);
  }
  if (indices) {
    for (const auto &index : index_set) indices->push_back(index);
  }
}

void Map::Downsample(double voxel_size) {
  auto indices = GetAllIndices();
  Downsample(indices, voxel_size);
}

void Map::Downsample(const std::vector<Index> &indices, double voxel_size) {
  for (const auto &index : indices) {
    if (!CheckIndex(index)) continue;
    common::VoxelDownSample<TPoint>(this->at(index), this->at(index).get(),
                                    voxel_size);
  }
}

Grid &Map::at(int z_idx) {
  return map_.at(z_idx);
}

const Grid &Map::at(int z_idx) const {
  return map_.at(z_idx);
}

Row &Map::at(int z_idx, int y_idx) {
  return map_.at(z_idx).at(y_idx);
}

const Row &Map::at(int z_idx, int y_idx) const {
  return map_.at(z_idx).at(y_idx);
}

std::vector<Index> Map::GetSubmapIndices(
    const TPoint &point, const std::vector<int> &submap_shapes) const {
  std::vector<Index> indices;
  Index index = GetIndex(point);
  int nz = submap_shapes[0] / 2, ny = submap_shapes[1] / 2,
      nx = submap_shapes[2] / 2;
  for (int k = -nz; k <= nz; ++k) {
    int idx_k = index.k + k;
    if (idx_k < 0 || idx_k >= shape_[0]) continue;
    for (int j = -ny; j <= ny; ++j) {
      int idx_j = index.j + j;
      if (idx_j < 0 || idx_j >= shape_[1]) continue;
      for (int i = -nx; i <= nx; ++i) {
        int idx_i = index.i + i;
        if (idx_i < 0 || idx_i >= shape_[2]) continue;
        indices.emplace_back(idx_k, idx_j, idx_i);
      }
    }
  }
  return indices;
}

std::vector<Index> Map::GetAllIndices() const {
  std::vector<Index> indices;
  for (int k = 0; k <= shape_[0]; ++k) {
    for (int j = 0; j <= shape_[1]; ++j) {
      for (int i = 0; i <= shape_[2]; ++i) {
        indices.emplace_back(k, j, i);
      }
    }
  }
  return indices;
}

Row::Row(int n) {
  row_.resize(n);
  for (auto &cloud : row_) cloud.reset(new TPointCloud);
}

TPointCloudPtr &Row::at(int idx) {
  return row_.at(idx);
}

const TPointCloudPtr &Row::at(int idx) const {
  return row_.at(idx);
}

void Row::clear() {
  for (auto &cloud : row_) cloud->clear();
}

TPointCloudPtr Row::GetAllPoints() const {
  TPointCloudPtr cloud_all(new TPointCloud);
  for (const auto &cloud : row_) *cloud_all += *cloud;
  return cloud_all;
}

Grid::Grid(int m, int n) {
  for (int i = 0; i < m; ++i) grid_.emplace_back(n);
}

void Grid::clear() {
  for (auto &row : grid_) row.clear();
}

Row &Grid::at(int idx) {
  return grid_.at(idx);
}

const Row &Grid::at(int idx) const {
  return grid_.at(idx);
}

TPointCloudPtr Grid::GetAllPoints() const {
  TPointCloudPtr cloud_all(new TPointCloud);
  for (const auto &row : grid_) *cloud_all += *row.GetAllPoints();
  return cloud_all;
}

}  // namespace oh_my_loam