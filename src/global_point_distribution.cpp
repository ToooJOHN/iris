#include "global_point_distribution.hpp"

namespace vllm
{

void GPD::init(size_t _N, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float gain)
{
  N = _N;
  data.resize(N);
  for (size_t i = 0; i < N; ++i) {
    data[i].resize(N);
    for (size_t j = 0; j < N; ++j) {
      data[i][j].resize(N);
    }
  }

  pcl::PointXYZ min_point, max_point;
  pcl::getMinMax3D(*cloud, min_point, max_point);

  segment = getResolution(min_point, max_point, N);
  bottom << min_point.x, min_point.y, min_point.z;
  top << max_point.x, max_point.y, max_point.z;

  // cut out pointcloud for each voxel
  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      for (size_t k = 0; k < N; k++) {
        Eigen::Vector3f min_box, max_box;
        min_box << segment.x() * static_cast<float>(i), segment.y() * static_cast<float>(j), segment.z() * static_cast<float>(k);
        max_box << segment.x() * static_cast<float>(i + 1), segment.y() * static_cast<float>(j + 1), segment.z() * static_cast<float>(k + 1);
        min_box += bottom;
        max_box += bottom;

        pcXYZ cloud_cropped;
        pcl::CropBox<pcl::PointXYZ> clop;
        clop.setInputCloud(cloud);
        clop.setMin(Eigen::Vector4f(min_box.x(), min_box.y(), min_box.z(), 1.0f));
        clop.setMax(Eigen::Vector4f(max_box.x(), max_box.y(), max_box.z(), 1.0f));
        clop.filter(cloud_cropped);

        // std::cout << i << " " << j << " " << k << " " << cloud_cropped.size() << std::endl;
        LPD tmp(cloud_cropped.makeShared(), gain);
        data[i][j][k] = tmp;
      }
    }
  }
}

LPD GPD::getLPD(const pcl::PointXYZ& point)
{
  Eigen::Vector3i index = getIndex(point);
  return data[index.x()][index.y()][index.z()];
}

Eigen::Vector3i GPD::getIndex(const pcl::PointXYZ& point)
{
  float x = point.x - bottom.x();
  float y = point.y - bottom.y();
  float z = point.z - bottom.z();
  x = std::max(std::min(x / segment.x(), static_cast<float>(N - 1)), 0.0f);
  y = std::max(std::min(y / segment.y(), static_cast<float>(N - 1)), 0.0f);
  z = std::max(std::min(z / segment.z(), static_cast<float>(N - 1)), 0.0f);

  return Eigen::Vector3i(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z));
}

Eigen::Vector3f GPD::getResolution(pcl::PointXYZ min, pcl::PointXYZ max, size_t N)
{
  float inv = 1.0f / static_cast<float>(N);
  return Eigen::Vector3f((max.x - min.x) * inv, (max.y - min.y) * inv, (max.z - min.z) * inv);
}
}  // namespace vllm