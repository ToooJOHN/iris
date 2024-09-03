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

//bridge_node.cpp 文件是 ROS 節點的主程序，它將 BridgeOpenVSLAM 類整合到 ROS 節點中，使得 VSLAM 系統可以通過 ROS 與其他模塊進行通信和協作。
//功能：
//ROS 節點初始化：設置並初始化 ROS 節點，讀取參數（如詞彙表路徑、配置文件路徑、影像話題名稱等），並準備 ROS 的訂閱和發布機制。
//影像訂閱和處理：從 ROS 中訂閱影像數據，將其傳遞給 BridgeOpenVSLAM 類進行處理，並根據 SLAM 系統的輸出進行後續操作。
//地標點雲發布：將從 BridgeOpenVSLAM 中獲得的地標點雲數據發布到 ROS 的指定話題，使得其他 ROS 節點可以使用這些數據。
//相機位姿發布：通過 publishPose 方法將相機的位姿發布到 ROS 中，使得其他節點可以了解相機在場景中的當前位置。
//主循環：控制節點的主循環，確保在 ROS 節點運行期間不斷處理新的影像數據並更新 SLAM 狀態。
//總結：
//bridge_node.cpp 是一個 ROS 節點，它將 BridgeOpenVSLAM 類的功能整合到 ROS 環境中，負責處理影像數據，並將結果（如地標點雲、相機位姿）發布到 ROS 網絡上，以便其他 ROS 節點可以使用這些信息。
#include "bridge.hpp"
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <iostream>
#include <pcl/correspondence.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void publishPose(const Eigen::Matrix4f& T, const std::string& child_frame_id)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setFromOpenGLMatrix(T.cast<double>().eval().data());
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", child_frame_id));
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "openvslam_bridge_node");

  // Get rosparams
  ros::NodeHandle pnh("~");
  bool is_image_compressed;
  int recollection = 30;
  std::string vocab_path, vslam_config_path, image_topic_name;
  pnh.getParam("vocab_path", vocab_path);
  pnh.getParam("vslam_config_path", vslam_config_path);
  pnh.getParam("image_topic_name0", image_topic_name);
  pnh.getParam("is_image_compressed", is_image_compressed);
  pnh.getParam("keyframe_recollection", recollection);
  ROS_INFO("vocab_path: %s, vslam_config_path: %s, image_topic_name: %s, is_image_compressed: %d",
      vocab_path.c_str(), vslam_config_path.c_str(), image_topic_name.c_str(), is_image_compressed);


  const int lower_threshold_of_points = 1500;
  const int upper_threshold_of_points = 2000;

  // Setup subscriber
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Time subscribed_stamp;
  cv::Mat subscribed_image;
  image_transport::TransportHints hints("raw");
  if (is_image_compressed) hints = image_transport::TransportHints("compressed");

  auto callback = [&subscribed_image, &subscribed_stamp](const sensor_msgs::ImageConstPtr& msg) -> void {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    subscribed_image = cv_ptr->image.clone();
    subscribed_stamp = msg->header.stamp;
  };


  image_transport::Subscriber image_subscriber = it.subscribe(image_topic_name, 5, callback, ros::VoidPtr(), hints);

  // Setup publisher
  ros::Publisher vslam_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZINormal>>("iris/vslam_data", 1);
  image_transport::Publisher image_publisher = it.advertise("iris/processed_image", 5);

  // Setup for OpenVSLAM
  iris::BridgeOpenVSLAM bridge;
  bridge.setup(vslam_config_path, vocab_path);

  std::chrono::system_clock::time_point m_start;
  ros::Rate loop_rate(20);
  float accuracy = 0.5f;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr vslam_data(new pcl::PointCloud<pcl::PointXYZINormal>);
  std::ofstream ofs_time("vslam_time.csv");

  // Start main loop
  ROS_INFO("start main loop.");
  while (ros::ok()) {
    if (!subscribed_image.empty()) {
      m_start = std::chrono::system_clock::now();  // start timer
      ros::Time process_stamp = subscribed_stamp;

      // process OpenVSLAM
      bridge.execute(subscribed_image);
      bridge.setCriteria(recollection, accuracy);
      bridge.getLandmarksAndNormals(vslam_data, std::numeric_limits<float>::max());

      // Reset input
      subscribed_image = cv::Mat();

      // Update threshold to adjust the number of points
      if (vslam_data->size() < lower_threshold_of_points && accuracy > 0.10) accuracy -= 0.01f;
      if (vslam_data->size() > upper_threshold_of_points && accuracy < 0.90) accuracy += 0.01f;
      std::cout << "accuracy: " << accuracy << std::endl;
      {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bridge.getFrame()).toImageMsg();
        image_publisher.publish(msg);
      }
      {
        pcl_conversions::toPCL(process_stamp, vslam_data->header.stamp);
        vslam_data->header.frame_id = "world";
        vslam_publisher.publish(vslam_data);
      }

      // Inform processing time
      std::stringstream ss;
      long time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_start).count();
      ss << "processing time= \033[35m"
         << time_ms
         << "\033[m ms";
      ofs_time << time_ms << std::endl;
      ROS_INFO("%s", ss.str().c_str());
    }
    publishPose(bridge.getCameraPose().inverse(), "iris/vslam_pose");

    // Spin and wait
    loop_rate.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Finalize openvslam_bridge::bridge_node");
  return 0;
}