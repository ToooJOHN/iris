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

#include "core/types.hpp"
#include "map/map.hpp"
#include "publish/publish.hpp"
#include "system/system.hpp"
#include <chrono>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


//
Eigen::Matrix4f listenTransform(tf::TransformListener& listener);

//
pcl::PointCloud<pcl::PointXYZINormal>::Ptr vslam_data(new pcl::PointCloud<pcl::PointXYZINormal>);
bool vslam_update = false;
void callback(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& msg)
{
  *vslam_data = *msg;
  if (vslam_data->size() > 0)
    vslam_update = true;
}

//
Eigen::Matrix4f T_recover = Eigen::Matrix4f::Zero();//T_recover 的作用是用來根據 /initialpose 消息設置或恢復機器人的初始位姿
pcl::PointCloud<pcl::PointXYZ>::Ptr whole_pointcloud = nullptr;
void callbackForRecover(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)//當 ROS 節點接收到來自 /initialpose 話題的消息時，回調函數 callbackForRecover 會被觸發,initialpose = msg
{
  ROS_INFO("/initial_pose is subscribed");

  float x = static_cast<float>(msg->pose.pose.position.x);
  float y = static_cast<float>(msg->pose.pose.position.y);
  float qw = static_cast<float>(msg->pose.pose.orientation.w);
  float qz = static_cast<float>(msg->pose.pose.orientation.z);

  float z = std::numeric_limits<float>::max();

  if (whole_pointcloud == nullptr) {//如果 whole_pointcloud 是空的（即尚未加載地圖），程式將 z 坐標設置為 0
    std::cout << "z=0 because whole_pointcloud is nullptr" << std::endl;
    z = 0;
  } else {//如果 whole_pointcloud 不為空，則程式會檢查點雲數據中的點，根據接收到的 (x, y) 坐標搜尋周圍的點雲數據，並使用 z 值最小的點來更新 z 坐標
    for (const pcl::PointXYZ& p : *whole_pointcloud) {
      constexpr float r2 = 5 * 5;  // [m^2]
      float dx = x - p.x;
      float dy = y - p.y;
      if (dx * dx + dy * dy < r2) {
        z = std::min(z, p.z);
      }
    }
  }
//程式會將 (x, y, z) 平移數據和計算出的旋轉角度（繞 z 軸的旋轉）組合，生成一個 4x4 的齊次變換矩陣 T_recover。這個矩陣表示機器人在空間中的位置和方向
  T_recover.setIdentity();//使用 Eigen::Matrix4f::Identity() 構建單位矩陣
  T_recover(0, 3) = x;//將位置數據 (x, y, z) 填入
  T_recover(1, 3) = y;
  T_recover(2, 3) = z;
  float theta = 2 * std::atan2(qz, qw);//使用 qw 和 qz 計算出旋轉角度 theta，並生成旋轉矩陣
  Eigen::Matrix3f R;
  R << 0, 0, 1,
      -1, 0, 0,
      0, -1, 0;
  T_recover.topLeftCorner(3, 3) = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()).toRotationMatrix() * R;//將旋轉矩陣與常數矩陣 R 組合，最後將結果填入 T_recover 的左上角，表示機器人的方向
  std::cout << "T_recover:\n"
            << T_recover << std::endl;
}

void writeCsv(std::ofstream& ofs, const ros::Time& timestamp, const Eigen::Matrix4f& iris_pose)
{
  auto convert = [](const Eigen::MatrixXf& mat) -> Eigen::VectorXf {
    Eigen::MatrixXf tmp = mat.transpose();
    return Eigen::VectorXf(Eigen::Map<Eigen::VectorXf>(tmp.data(), mat.size()));
  };

  ofs << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10);
  ofs << timestamp.toSec() << " " << convert(iris_pose).transpose() << std::endl;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "iris_node");
  ros::NodeHandle nh;

  // Setup subscriber
  ros::Subscriber vslam_subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZINormal>>("iris/vslam_data", 5, callback);
  ros::Subscriber recover_pose_subscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 5, callbackForRecover);
  tf::TransformListener listener;

  // Setup publisher
  ros::Publisher target_pc_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("iris/target_pointcloud", 1, true);
  ros::Publisher whole_pc_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("iris/whole_pointcloud", 1, true);
  ros::Publisher source_pc_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("iris/source_pointcloud", 1);
  ros::Publisher iris_path_publisher = nh.advertise<nav_msgs::Path>("iris/iris_path", 1);
  ros::Publisher vslam_path_publisher = nh.advertise<nav_msgs::Path>("iris/vslam_path", 1);
  ros::Publisher correspondences_publisher = nh.advertise<visualization_msgs::Marker>("iris/correspondences", 1);
  ros::Publisher scale_publisher = nh.advertise<std_msgs::Float32>("iris/align_scale", 1);
  ros::Publisher processing_time_publisher = nh.advertise<std_msgs::Float32>("iris/processing_time", 1);
  // ros::Publisher normal_publisher = nh.advertise<visualization_msgs::MarkerArray>("iris/normals", 1);
  // ros::Publisher covariance_publisher = nh.advertise<visualization_msgs::MarkerArray>("iris/covariances", 1);
  iris::Publication publication;

  // Get rosparams
  ros::NodeHandle pnh("~");
  std::string config_path, pcd_path;
  pnh.getParam("iris_config_path", config_path);
  pnh.getParam("pcd_path", pcd_path);
  ROS_INFO("config_path: %s, pcd_path: %s", config_path.c_str(), pcd_path.c_str());

  // Initialize config
  iris::Config config(config_path);

  // Load LiDAR map
  iris::map::Parameter map_param(
      pcd_path, config.voxel_grid_leaf, config.normal_search_leaf, config.submap_grid_leaf);
  std::shared_ptr<iris::map::Map> map = std::make_shared<iris::map::Map>(map_param, config.T_init);

  // Initialize system
  std::shared_ptr<iris::System> system = std::make_shared<iris::System>(config, map);

  std::chrono::system_clock::time_point m_start;
  Eigen::Matrix4f offseted_vslam_pose = config.T_init;
  Eigen::Matrix4f iris_pose = config.T_init;

  // Publish map
  iris::publishPointcloud(whole_pc_publisher, map->getSparseCloud());
  iris::publishPointcloud(target_pc_publisher, map->getTargetCloud());
  whole_pointcloud = map->getSparseCloud();
  std::ofstream ofs_track("trajectory.csv");
  std::ofstream ofs_time("iris_time.csv");

  iris::map::Info last_map_info;

  // Start main loop
  ros::Rate loop_rate(20);
  ROS_INFO("start main loop.");
  while (ros::ok()) {

    Eigen::Matrix4f T_vslam = listenTransform(listener);//調用 listenTransform 函數，從 TF（坐標變換系統）中獲取機器人的 VSLAM（視覺同步定位與建圖）位姿變換矩陣，並將其存儲在 T_vslam 變數中
    if (!T_recover.isZero()) {//當系統檢測到 T_recover 不是零矩陣時，程式會將該矩陣應用到系統中，將機器人的世界位姿（T_world）設置為 T_recover，從而恢復機器人的位置和方向
      std::cout << "apply recover pose" << std::endl;
      system->specifyTWorld(T_recover);
      T_recover.setZero();//T_recover 被重置為零，防止重複應用
    }

    if (vslam_update) {
      vslam_update = false;
      m_start = std::chrono::system_clock::now();//这里是使用了 C++ 的标准库 std::chrono 来获取当前系统时间。m_start 是一个时间点，用来记录当前的时间戳。
      ros::Time process_stamp;
      pcl_conversions::fromPCL(vslam_data->header.stamp, process_stamp);//它将 PCL 点云的时间戳 vslam_data->header.stamp 转换为 ROS 格式的时间，并存储在 process_stamp 中

      // Execution
      system->execute(2, T_vslam, vslam_data);

      // Publish for rviz
      system->popPublication(publication);
      iris::publishPointcloud(source_pc_publisher, publication.cloud);
      iris::publishPath(iris_path_publisher, publication.iris_trajectory);
      iris::publishPath(vslam_path_publisher, publication.offset_trajectory);
      iris::publishCorrespondences(correspondences_publisher, publication.cloud, map->getTargetCloud(), publication.correspondences);
      // iris::publishNormal(normal_publisher, publication.cloud, publication.normals);
      // iris::publishCovariance(covariance_publisher, publication.cloud, publication.normals);

      if (last_map_info != map->getLocalmapInfo()) {
        iris::publishPointcloud(target_pc_publisher, map->getTargetCloud());
      }
      last_map_info = map->getLocalmapInfo();
      std::cout << "map: " << last_map_info.toString() << std::endl;


      // Processing time
      long time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_start).count();
      std::stringstream ss;
      ss << "processing time= \033[35m"
         << time_ms
         << "\033[m ms";
      ofs_time << time_ms << std::endl;
      ROS_INFO("Iris/ALIGN: %s", ss.str().c_str());
      {
        std_msgs::Float32 scale;
        scale.data = iris::util::getScale(publication.T_align);
        scale_publisher.publish(scale);

        std_msgs::Float32 processing_time;
        processing_time.data = static_cast<float>(time_ms);
        processing_time_publisher.publish(processing_time);
      }

      offseted_vslam_pose = publication.offset_camera;
      iris_pose = publication.iris_camera;

      writeCsv(ofs_track, process_stamp, iris_pose);
    }

    iris::publishPose(offseted_vslam_pose, "iris/offseted_vslam_pose");
    iris::publishPose(iris_pose, "iris/iris_pose");


    // Spin and wait
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Finalize the system");
  return 0;
}


Eigen::Matrix4f listenTransform(tf::TransformListener& listener)
{
  tf::StampedTransform transform;// 用來存儲從TF中獲取的變換
  try {
    listener.lookupTransform("world", "iris/vslam_pose", ros::Time(0), transform); // 從 "world" 到 "iris/vslam_pose" 查詢當前的變換矩陣
  } catch (...) {
  }

  double data[16];// 將查詢到的變換轉換為 4x4 的 OpenGL 矩陣格式
  transform.getOpenGLMatrix(data);// 以OpenGL矩陣的形式獲取變換數據
  Eigen::Matrix4d T(data);// 用 4x4 的矩陣表示這個變換
  return T.cast<float>(); // 將矩陣類型轉換為 float 並返回
}