#pragma once

#include <vector>

#include "common/common.h"
#include "oh_my_loam/base/types.h"

namespace oh_my_loam {

class Row;

class Grid;

struct Index {
  int k, j, i;
  Index() = default;
  Index(int k, int j, int i) : k(k), j(j), i(i) {}

  struct Comp {
    bool operator()(const Index &idx1, const Index &idx2) const {
      return (idx1.k < idx2.k) || (idx1.k == idx2.k && idx1.j < idx2.j) ||
             (idx1.k == idx2.k && idx1.j == idx2.j && idx1.i < idx2.i);
    }
  };
};

class Map {
 public:
  Map(const std::vector<int> &shape, const std::vector<double> &step);

  Map(const std::vector<int> &shape, double step);

  TPointCloudPtr &at(const Index &index);

  const TPointCloudPtr &at(const Index &index) const;

  void clear();

  void ShiftZ(int n);

  void ShiftY(int n);

  void ShiftX(int n);

  const std::vector<int> shape() const {
    return std::vector<int>(shape_, shape_ + 3);
  }

  Index GetIndex(const TPoint &point) const;

  bool CheckIndex(const Index &index) const;

  TPointCloudPtr GetSubmapPoints(const TPoint &point,
                                 const std::vector<int> &submap_shapes) const;

  TPointCloudPtr GetAllPoints() const;

  void AddPoints(const TPointCloudConstPtr &cloud,
                 std::vector<Index> *const indices = nullptr);

  void Downsample(double voxel_size);

  void Downsample(const std::vector<Index> &indices, double voxel_size);

 private:
  Grid &at(int z_idx);

  const Grid &at(int z_idx) const;

  Row &at(int z_idx, int y_idx);

  const Row &at(int z_idx, int y_idx) const;

  TPointCloudPtr &at(int z_idx, int y_idx, int x_idx);

  const TPointCloudPtr &at(int z_idx, int y_idx, int x_idx) const;

  std::vector<Index> GetSubmapIndices(
      const TPoint &point, const std::vector<int> &submap_shapes) const;

  std::vector<Index> GetAllIndices() const;

  int shape_[3], center_[3];  // order: z, y, x

  double step_[3];

  std::vector<Grid> map_;

  DISALLOW_COPY_AND_ASSIGN(Map);
};

class Row {
 public:
  explicit Row(int n);

  TPointCloudPtr &at(int idx);

  const TPointCloudPtr &at(int idx) const;

  void clear();

  TPointCloudPtr GetAllPoints() const;

 private:
  std::vector<TPointCloudPtr> row_;
};

class Grid {
 public:
  Grid(int m, int n);

  void clear();

  Row &at(int idx);

  const Row &at(int idx) const;

  TPointCloudPtr GetAllPoints() const;

 private:
  std::vector<Row> grid_;
};

}  // namespace oh_my_loam