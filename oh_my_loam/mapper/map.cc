#include "oh_my_loam/mapper/map.h"

#include "common/log/log.h"

namespace oh_my_loam {

Row::Row(int n) {
  row_.resize(n);
  for (auto &cloud : row_) cloud.reset(new TPointCloud);
}

TPointCloudPtr &Row::at(int ix) {
  return row_.at(ix);
}

const TPointCloudPtr &Row::at(int ix) const {
  return row_.at(ix);
}

void Row::Clear() {
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

void Grid::Clear() {
  for (auto &row : grid_) row.Clear();
}

Row &Grid::at(int ix) {
  return grid_.at(ix);
}

const Row &Grid::at(int ix) const {
  return grid_.at(ix);
}

TPointCloudPtr Grid::GetAllPoints() const {
  TPointCloudPtr cloud_all(new TPointCloud);
  for (const auto &row : grid_) *cloud_all += *row.GetAllPoints();
  return cloud_all;
}

Map::Map(int z_size, int y_size, int x_size, double z_step, double y_step,
         double x_step)
    : z_size_(z_size),
      y_size_(y_size),
      x_size_(x_size),
      z_step_(z_step),
      y_step_(y_step),
      x_step_(x_step),
      z_center_(x_size / 2),
      y_center_(y_size / 2),
      x_center_(x_size / 2) {
  ACHECK(z_size % 2 == 1 && y_size % 2 == 1 && x_size % 2 == 1);
  ACHECK(z_step_ > 0.0 && y_step_ > 0.0 && x_step_ > 0.0);
  for (int i = 0; i < z_size; ++i) map_.emplace_back(y_size, x_size);
}

Grid &Map::at(int z_ix) {
  return map_.at(z_ix);
}

const Grid &Map::at(int z_ix) const {
  return map_.at(z_ix);
}

Row &Map::at(int z_ix, int y_ix) {
  return map_.at(z_ix).at(y_ix);
}

const Row &Map::at(int z_ix, int y_ix) const {
  return map_.at(z_ix).at(y_ix);
}

TPointCloudPtr &Map::at(int z_ix, int y_ix, int x_ix) {
  return map_.at(z_ix).at(y_ix).at(x_ix);
}

const TPointCloudPtr &Map::at(int z_ix, int y_ix, int x_ix) const {
  return map_.at(z_ix).at(y_ix).at(x_ix);
}

TPointCloudPtr &Map::at(const Index &index) {
  return map_.at(index.k).at(index.j).at(index.i);
}

const TPointCloudPtr &Map::at(const Index &index) const {
  return map_.at(index.k).at(index.j).at(index.i);
}

void Map::Clear() {
  for (auto &grid : map_) grid.Clear();
}

void Map::ShiftZ(int n) {
  if (n == 0) return;
  if (n > 0) {
    for (int k = z_size_ - 1; k >= n; --k) {
      std::swap(this->at(k), this->at(k - n));
    }
    for (int k = 0; k < n; ++k) this->at(k).Clear();
  } else {
    for (int k = 0; k < z_size_ - n; ++k) {
      std::swap(this->at(k), this->at(k + n));
    }
    for (int k = z_size_ - n - 1; k < z_size_; ++k) {
      this->at(k).Clear();
    }
  }
  z_center_ -= n;
}

void Map::ShiftY(int n) {
  if (n == 0) return;
  for (int k = 0; k < z_size_; ++k) {
    if (n > 0) {
      for (int j = y_size_ - 1; j >= n; --j) {
        std::swap(this->at(k, j), this->at(k, j - n));
      }
      for (int j = 0; j < n; ++j) this->at(k, j).Clear();
    } else {
      for (int j = 0; j < y_size_ - n; ++j) {
        std::swap(this->at(k, j), this->at(k, j + n));
      }
      for (int j = y_size_ - n - 1; j < y_size_; ++j) {
        this->at(k, j).Clear();
      }
    }
  }
  y_center_ -= n;
}

void Map::ShiftX(int n) {
  if (n == 0) return;
  for (int k = 0; k < z_size_; ++k) {
    for (int j = 0; j < y_size_; ++j) {
      if (n > 0) {
        for (int i = x_size_ - 1; i >= n; --i) {
          std::swap(this->at(k, j, i), this->at(k, j, i - n));
        }
        for (int i = 0; i < n; ++i) this->at(k, j, i)->clear();

      } else {
        for (int i = 0; i < x_size_ - n; ++i) {
          std::swap(this->at(k, j, i), this->at(k, j, i + n));
        }
        for (int i = x_size_ - n - 1; i < x_size_; ++i) {
          this->at(k, j, i)->clear();
        }
      }
    }
  }
  x_center_ -= n;
}

Index Map::GetIndex(const TPoint &point) {
  Index index;
  index.k = static_cast<int>(point.x / z_step_) + z_center_;
  index.j = static_cast<int>(point.x / y_step_) + y_center_;
  index.i = static_cast<int>(point.x / x_step_) + x_center_;
  return index;
}

std::vector<Index> Map::GetSurrIndices(const TPoint &point, int n) {
  std::vector<Index> indices;
  Index index = GetIndex(point);
  for (int k = -n; k <= n; ++k) {
    for (int j = -n; j <= n; ++j) {
      for (int i = -n; i <= n; ++i) {
        indices.emplace_back(index.k + k, index.j + j, index.i + i);
      }
    }
  }
  return indices;
}

TPointCloudPtr Map::GetSurrPoints(const TPoint &point, int n) {
  TPointCloudPtr cloud_all(new TPointCloud);
  for (const auto &index : GetSurrIndices(point, n)) {
    *cloud_all += *this->at(index);
  }
  return cloud_all;
}

TPointCloudPtr Map::GetAllPoints() const {
  TPointCloudPtr cloud_all(new TPointCloud);
  for (const auto &grid : map_) *cloud_all += *grid.GetAllPoints();
  return cloud_all;
}

}  // namespace oh_my_loam