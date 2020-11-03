#pragma once

namespace oh_my_loam {

using PCLVisualizer = pcl::visualization::PCLVisualizer;
#define PCLColorHandlerCustom pcl::visualization::PointCloudColorHandlerCustom
#define PCLColorHandlerGenericField \
  pcl::visualization::PointCloudColorHandlerGenericField

template <typename PointT>
void DrawPointCloud(const pcl::PointCloud<PointT>& cloud, const Color& color,
                    const std::string& id, PCLVisualizer* const viewer,
                    int pt_size = 3) {
  PCLColorHandlerCustom<PointT> color_handler(cloud.makeShared(), color.r,
                                              color.g, color.b);
  viewer->addPointCloud<PointT>(cloud.makeShared(), color_handler, id);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_size, id);
}

template <typename PointT>
void DrawPointCloud(const pcl::PointCloud<PointT>& cloud,
                    const std::string& field, const std::string& id,
                    PCLVisualizer* const viewer, int pt_size = 3) {
  PCLColorHandlerGenericField<PointT> color_handler(cloud.makeShared(), field);
  viewer->addPointCloud<PointT>(cloud.makeShared(), color_handler, id);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_size, id);
}

template <typename PointT>
void DrawLine(const PointT& pt1, const PointT& pt2, const Color& color,
              const std::string& id, PCLVisualizer* const viewer) {
  viewer->addLine(pt1, pt2, color.r, color.g, color.b, id);
}

}  // namespace oh_my_loam