// Copyright (c) 2020, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "map/map.hpp"
#include "core/util.hpp"
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <utility>

namespace iris
{

namespace map
{
Map::Map(const Parameter& parameter, const Eigen::Matrix4f& T_init)//这段代码定义了 Map 类的构造函数 Map::Map，并通过初始化列表对类的成员变量进行初始化
    : cache_file("iris.cache"), parameter(parameter),
      local_target_cloud(new pcXYZ),
      local_target_normals(new pcNormal)
{
  bool recalculation_is_necessary = isRecalculationNecessary();

  if (!recalculation_is_necessary) {
    all_target_cloud = pcXYZ::Ptr(new pcXYZ); //创建一个智能指针对象，并将它指向新创建的 pcXYZ 对象
    all_target_cloud = pcXYZ::Ptr(new pcXYZ); 
    all_target_normals = pcNormal::Ptr(new pcNormal);
    all_sparse_cloud = pcXYZ::Ptr(new pcXYZ);

    // load cache file
    bool flag1 = (pcl::io::loadPCDFile<pcl::PointXYZ>(cache_cloud_file, *all_target_cloud) != -1);
    bool flag2 = (pcl::io::loadPCDFile<pcl::Normal>(cache_normals_file, *all_target_normals) != -1);
    bool flag3 = (pcl::io::loadPCDFile<pcl::PointXYZ>(cache_sparse_file, *all_sparse_cloud) != -1);

    if (flag1 && flag2 && flag3)
      std::cout << "Because of cache hit, cached target point cloud was loaded" << std::endl;
    else
      recalculation_is_necessary = true;
  }

  if (recalculation_is_necessary) {
    std::cout << "Because of cache miss, recalculate the target point cloud" << std::endl;

    std::cout << "start loading pointcloud & esitmating normal with leafsize " << parameter.voxel_grid_leaf << " search_radius " << parameter.normal_search_radius << std::endl;
    all_target_cloud = pcXYZ::Ptr(new pcXYZ);
    all_target_normals = pcNormal::Ptr(new pcNormal);
    all_sparse_cloud = pcXYZ::Ptr(new pcXYZ);
    util::loadMap(parameter.pcd_file, all_target_cloud, all_target_normals, parameter.voxel_grid_leaf, parameter.normal_search_radius);

    {
      pcl::VoxelGrid<pcl::PointXYZ> filter;// 創建一個體素濾波器對象
      filter.setInputCloud(all_target_cloud);// 設置要進行濾波處理的輸入點雲（all_target_cloud）
      filter.setLeafSize(4 * parameter.voxel_grid_leaf, 4 * parameter.voxel_grid_leaf, 4 * parameter.voxel_grid_leaf);// 設置濾波器的葉子大小為原大小的四倍，以進行稀疏化
      filter.filter(*all_sparse_cloud); // 對點雲進行濾波，結果存儲在 all_sparse_cloud 中
    }


    // save as cache file
    std::cout << "save pointcloud" << std::endl;
    pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZ>(cache_cloud_file, *all_target_cloud);
    pcl::io::savePCDFileBinaryCompressed<pcl::Normal>(cache_normals_file, *all_target_normals);
    pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZ>(cache_sparse_file, *all_sparse_cloud);

    // update cache information
    std::ofstream ofs(cache_file);
    ofs << parameter.toString();
  }
  std::cout << "all_target_cloud_size " << all_target_cloud->size() << std::endl;

  // Calculate the number of submap and its size
  std::cout << "It starts making submaps. This may take few seconds." << std::endl;
  float L = parameter.submap_grid_leaf;
  if (L < 1) {
    L = 1;
    std::cout << "please set positive number for parameter.submap_grid_leaf" << std::endl;
  }

  // Make submaps
  for (size_t i = 0; i < all_target_cloud->size(); i++) {//使用 for 循环遍历 all_target_cloud 中的每个点，all_target_cloud->size() 返回点云中点的总数。
    pcl::PointXYZ p = all_target_cloud->at(i);//对于每个点，分别获取其对应的点云坐标 p 和法向量 n
    pcl::Normal n = all_target_normals->at(i);//对于每个点，分别获取其对应的点云坐标 p 和法向量 n

    int id_x = static_cast<int>(std::floor(p.x / L));//通过将 x 和 y 坐标除以 L，我们将点的实际坐标转换为子地图单元的索引
    int id_y = static_cast<int>(std::floor(p.y / L));//通过将 x 和 y 坐标除以 L，我们将点的实际坐标转换为子地图单元的索引

    std::pair key = std::make_pair(id_x, id_y);//键值对的生成：使用 id_x 和 id_y 生成一个唯一的 key，用来标识子地图单元
    submap_cloud[key].push_back(p);//submap_cloud[key]：使用 key 作为索引，访问或创建一个子地图单元的点云集合。如果 submap_cloud 中还没有这个 key，那么它会自动创建一个新的条目
    submap_normals[key].push_back(n);//push_back(p)：将点 p 添加到 submap_cloud[key] 对应的点云集合中
  }

  // Construct local map
  updateLocalmap(T_init);
}

bool Map::isRecalculationNecessary() const
{
  std::ifstream ifs(cache_file);
  // If cahce data doesn't exist, recalculate
  if (!ifs)
    return true;
  std::string data;

  // If cahce data doesn't match with parameter, recalculate
  std::getline(ifs, data);
  if (data != parameter.toString())
    return true;

  return false;
}

bool Map::informCurrentPose(const Eigen::Matrix4f& T)
{
  bool is_necessary = isUpdateNecessary(T);
  if (!is_necessary)
    return false;

  updateLocalmap(T);
  return true;
}

bool Map::isUpdateNecessary(const Eigen::Matrix4f& T) const
{
  // NOTE: The boundaries of the submap have overlaps in order not to vibrate

  // (1) Condition about the location
  float distance = (T.topRightCorner(2, 1) - localmap_info.xy()).cwiseAbs().maxCoeff();
  if (distance > 0.75 * parameter.submap_grid_leaf) {
    std::cout << "map update because of the distance condition" << std::endl;
    return true;
  }

  // (2) Condition about the location
  float yaw = yawFromPose(T);
  if (subtractAngles(yaw, localmap_info.theta) > 60.f / 180.f * 3.14f) {
    std::cout << "map update because of the angle condition" << std::endl;
    return true;
  }


  // Then, it need not to update the localmap
  return false;
}

void Map::updateLocalmap(const Eigen::Matrix4f& T)//updateLocalmap 是 Map 类的成员函数，接受一个 Eigen::Matrix4f 类型的参数 T，表示当前的位姿矩阵
{
  std::cout << "\033[1;4;36m###############" << std::endl;
  std::cout << "Update Localmap" << std::endl;
  std::cout << "###############\033[m" << std::endl;

  Eigen::Vector3f t = T.topRightCorner(3, 1);//从位姿矩阵 T 中提取出右上角的 3x1 部分，这表示平移向量 t，即当前相机的位置信息
  const float L = parameter.submap_grid_leaf;
  int id_x = static_cast<int>(std::floor(t.x() / L));
  int id_y = static_cast<int>(std::floor(t.y() / L));
  //t.x() / L：将平移向量 t 的 x 坐标除以子地图单元的大小 L，确定当前点在 x 方向上的相对位置
  //std::floor(t.x() / L)：使用 std::floor 函数将结果向下取整，得到点在子地图网格中的整数索引
  //static_cast<int>(...)：将结果转换为整数，分别存储在 id_x 和 id_y 中，表示当前点在子地图中的 x 和 y 方向上的索引位置
  std::cout << "id_x " << id_x << " id_y " << id_y << std::endl;

  int pattern = static_cast<int>(yawFromPose(T) / (3.14f / 4.0f));
  //yawFromPose(T)：调用函数 yawFromPose 从位姿矩阵 T 中计算出当前的偏航角（即相对于北方向的角度）
  //yawFromPose(T) / (3.14f / 4.0f)：将偏航角除以 π/4，得到一个与当前方向模式相关的值（将圆周分为八个部分）
  int x_min, y_min, dx, dy;
  //x_min：局部地图在 x 方向上的起始子地图单元的索引
  //y_min：局部地图在 y 方向上的起始子地图单元的索引

  
  //为什么 x_min 和 y_min 是左上角？
  //x_min = id_x - <某个偏移量>：这是通过从当前所在位置 id_x 向左（减少 x 值）确定的。因此，x_min 是该区域在 x 方向上的最小值。
  //y_min = id_y - <某个偏移量>：同样地，这是通过从当前所在位置 id_y 向上（减少 y 值）确定的。因此，y_min 是该区域在 y 方向上的最小值。
  
  //dx：局部地图在 x 方向上覆盖的子地图单元的数量
  //dy：局部地图在 y 方向上覆盖的子地图单元的数量
  float new_info_theta;//声明 new_info_theta，用于存储新的角度信息
  switch (pattern) {//根据 pattern 的不同值选择不同的子地图范围和方向处理逻辑
  case 0:
  case 7:
    x_min = id_x - 1;
    y_min = id_y - 1;
    dx = 4;
    dy = 3;
    new_info_theta = 0;
    break;
  case 1:
  case 2:
    x_min = id_x - 1;
    y_min = id_y - 1;
    dx = 3;
    dy = 4;
    new_info_theta = 3.1415f * 0.5f;
    break;
  case 3:
  case 4:
    x_min = id_x - 2;
    y_min = id_y - 1;
    dx = 4;
    dy = 3;
    new_info_theta = 3.1415f;
    break;
  case 5:
  case 6:
  default:
    x_min = id_x - 1;
    y_min = id_y - 2;
    dx = 3;
    dy = 4;
    new_info_theta = 3.1415f * 1.5f;
    break;
  }

  // Critical section from here
  {
    local_target_cloud->clear();
    local_target_normals->clear();

    for (int i = 0; i < dx; i++) {       //遍历当前局部地图覆盖的所有子地图单元
      for (int j = 0; j < dy; j++) {
        std::pair<int, int> key = std::make_pair(x_min + i, y_min + j);//计算当前遍历到的子地图单元的索引
        if (submap_cloud.count(key) == 0) {//
          continue;
        }
        //submap_cloud.count(key)：检查在 submap_cloud 中是否存在 key 对应的子地图单元的数据
        //如果 count 返回 0，意味着 key 对应的子地图单元没有数据
        //如果当前子地图单元没有数据，则跳过当前循环，继续检查下一个子地图单元
        *local_target_cloud += submap_cloud[key];
        //如果 key 对应的子地图单元存在数据，这行代码将 submap_cloud[key] 中的点云数据累加到 local_target_cloud 中
        *local_target_normals += submap_normals[key];
      }
    }
  }
  {
    localmap_info.x = (static_cast<float>(id_x) + 0.5f) * L,
    localmap_info.y = (static_cast<float>(id_y) + 0.5f) * L,
    localmap_info.theta = new_info_theta;
    //更新局部地图（local map）的中心位置和方向角度 将 id_x 和 id_y 转换为真实坐标
    //static_cast<float>(id_x) 和 static_cast<float>(id_y)：将整数索引 id_x 和 id_y 转换为浮点数，以便进行后续的精确计算
    //+ 0.5f：将 id_x 和 id_y 加上 0.5f 是为了将坐标从子地图单元的左上角移动到子地图单元的中心位置
    //将索引乘以单元格大小 L，转换为真实的物理坐标
  }
  std::cout << "new map-info: "
            << localmap_info.x << ", "
            << localmap_info.y << ", "
            << localmap_info.theta
            << std::endl;
  // Critical section until here
}

float Map::yawFromPose(const Eigen::Matrix4f& T) const
{
  Eigen::Matrix3f R = util::normalizeRotation(T);

  // When the optical axis of the camera is pointing to the X-axis
  // and the upper side of the camera is pointing to the Z-axis,
  // the rotation matrix is as follows,
  Eigen::Matrix3f camera_rotate;
  camera_rotate << 0, 0, 1,
      -1, 0, 0,
      0, -1, 0;

  // Therefore, multiply the inverse rotation matrix of it.
  // To extract the rotation on the XY-plane, we calculate how a unit vector is moved by a remained rotation.
  Eigen::Vector3f direction = (R * camera_rotate.transpose()) * Eigen::Vector3f::UnitX();

  float theta = std::atan2(direction.y(), direction.x());  // [-pi,pi]
  if (theta < 0)
    return theta + 6.28f;
  return theta;
}

}  // namespace map
}  // namespace iris