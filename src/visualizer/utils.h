#pragma once

namespace oh_my_loam {

using PCLVisualizer = pcl::visualization::PCLVisualizer;
#define PCLColorHandlerCustom pcl::visualization::PointCloudColorHandlerCustom
#define PCLColorHandlerGenericField \
  pcl::visualization::PointCloudColorHandlerGenericField

template <typename PointType>
void AddPointCloud(const typename pcl::PointCloud<PointType>::ConstPtr& cloud,
                   const Color& color, const std::string& id,
                   PCLVisualizer* const viewer, int point_size = 3) {
  PCLColorHandlerCustom<PointType> color_handler(cloud, color.r, color.g,
                                                 color.b);
  viewer->addPointCloud<PointType>(cloud, color_handler, id);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
}

template <typename PointType>
void AddPointCloud(const typename pcl::PointCloud<PointType>::ConstPtr& cloud,
                   const std::string& field, const std::string& id,
                   PCLVisualizer* const viewer, int point_size = 3) {
  PCLColorHandlerGenericField<PointType> color_handler(cloud, field);
  viewer->addPointCloud<PointType>(cloud, color_handler, id);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
}

template <typename PointType>
void AddLine(const PointType& pt1, const PointType& pt2, const Color& color,
             const std::string& id, PCLVisualizer* const viewer) {
  viewer->addLine(pt1, pt2, color.r, color.g, color.b, id);
}

}  // namespace oh_my_loam