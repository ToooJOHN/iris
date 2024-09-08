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


//这段代码实现了一个类 BridgeOpenVSLAM，它与 OpenVSLAM 系统进行交互封裝了與 OpenVSLAM 系統的交互邏輯，为 iris 系统提供处理图像和获取地标点云信息的功能
//功能：
//LAM 系統初始化：BridgeOpenVSLAM::setup 方法負責初始化 OpenVSLAM 系統，載入配置文件和詞彙表文件，並啟動 SLAM 系統。
//地標點雲的獲取：BridgeOpenVSLAM::getLandmarksAndNormals 方法從 VSLAM 系統中獲取地標點雲，並將其轉換為 pcl::PointCloud<pcl::PointXYZINormal> 格式。
//相機位姿獲取：BridgeOpenVSLAM::getCameraPose 方法返回當前相機的位姿矩陣。
//圖像處理：BridgeOpenVSLAM::execute 方法接收輸入的圖像，將其餵入 SLAM 系統進行處理，並根據追踪狀態決定是否需要重置系統。
//狀態管理：包含系統重置和狀態獲取等功能，以確保 SLAM 系統的穩定運行。
//總結：
//bridge.cpp 是一個封裝層，提供與 OpenVSLAM 系統進行高級交互的接口。它負責處理來自 VSLAM 系統的低層數據並轉換成 iris 可以使用的格式。
#include "bridge.hpp"
#include <openvslam/config.h>
#include <openvslam/data/landmark.h>
#include <openvslam/publish/frame_publisher.h>
#include <openvslam/publish/map_publisher.h>

#include <chrono>
#include <iostream>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>

namespace iris
{
BridgeOpenVSLAM::~BridgeOpenVSLAM()
// ~在 C++ 中表示析構函數（destructor）。析構函數是一個特殊的成員函數，當一個對象的生命週期結束時會被自動調用，用來釋放資源或執行清理工作。
//這個析構函數的任務是確保當 BridgeOpenVSLAM 對象被銷毀時，SLAM 系統可以正常關閉，並且不會留下未完成的工作
{
  if (SLAM_ptr == nullptr)
    return;

  // wait until the loop BA is finished
  while (SLAM_ptr->loop_BA_is_running()) {
    std::this_thread::sleep_for(std::chrono::microseconds(5000));
  }

  // shutdown the SLAM process
  SLAM_ptr->shutdown();
}

void BridgeOpenVSLAM::setup(const std::string& config_path, const std::string& vocab_path)
{
  // setup logger
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
  spdlog::set_level(spdlog::level::info);

  // load configuration
  std::shared_ptr<openvslam::config> cfg;
  try {
    cfg = std::make_shared<openvslam::config>(config_path);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }

  // run tracking
  if (cfg->camera_->setup_type_ != openvslam::camera::setup_type_t::Monocular) {
    std::cout << "Invalid setup type: " + cfg->camera_->get_setup_type_string() << std::endl;
    exit(EXIT_FAILURE);
  }

  // build a SLAM system
  SLAM_ptr = std::make_shared<openvslam::system>(cfg, vocab_path);
  SLAM_ptr->startup();
  SLAM_ptr->disable_loop_detector();
}

void BridgeOpenVSLAM::getLandmarksAndNormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& vslam_data, float height) const
{
  auto t0 = std::chrono::system_clock::now();

  if (recollection == 0 || accuracy < 0) {
    std::cerr << "ERROR: recollection & accuracy are not set" << std::endl;
    exit(1);
  }

  std::set<openvslam::data::landmark*> local_landmarks;
  SLAM_ptr->get_map_publisher()->get_landmarks(local_landmarks);//从 OpenVSLAM 系统中获取当前的地标点，并将它们存储在 local_landmarks 集合中

  vslam_data->clear();

  if (local_landmarks.empty()) return;

  Eigen::Vector3d t_vslam = SLAM_ptr->get_map_publisher()->get_current_cam_pose().topRightCorner(3, 1);

  unsigned int max_id = SLAM_ptr->get_map_publisher()->get_max_keyframe_id();
  for (const auto local_lm : local_landmarks) {
    unsigned int first_observed_id = local_lm->first_keyfrm_id_;//获取地标点第一次被观测到的关键帧 ID
    unsigned int last_observed_id = local_lm->last_observed_keyfrm_id_;//获取地标点最后一次被观测到的关键帧 ID
    if (local_lm->will_be_erased()) continue;//如果地标点即将被删除（无效），则跳过此地标点
    if (local_lm->get_observed_ratio() < accuracy) continue;//如果地标点的观测比率低于预设的精度 accuracy，则跳过该地标点
    if (max_id > recollection && last_observed_id < max_id - recollection) continue;
  
    const openvslam::Vec3_t pos = local_lm->get_pos_in_world();//获取地标点在世界坐标系中的位置 pos
    // const openvslam::Vec3_t normal = local_lm->get_obs_mean_normal();

    // when the distance is 5m or more, the weight is minimum.
    // float weight = static_cast<float>(1.0 - (t_vslam - pos).norm() * 0.2);
    float weight = 1.0f;
    weight = std::min(std::max(weight, 0.1f), 1.0f);//对权重进行限制，确保权重在 0.1 到 1.0 之间

    Eigen::Vector3f t = getCameraPose().inverse().topRightCorner(3, 1);//获取当前相机的位姿（位置）并将其存储在 t 中
    if (pos.y() - t.y() < -height) continue;
    //如果地标点的高度 pos.y() 与相机高度 t.y() 的差值小于 -height（即地标点低于相机的高度阈值 height），则跳过此地标点。
    pcl::PointXYZINormal p;
    p.x = static_cast<float>(pos.x());
    p.y = static_cast<float>(pos.y());
    p.z = static_cast<float>(pos.z());
    // p.normal_x = static_cast<float>(normal.x());
    // p.normal_y = static_cast<float>(normal.y());
    // p.normal_z = static_cast<float>(normal.z());
    p.intensity = weight;
    vslam_data->push_back(p);
    //将通过过滤的地标点的位置和权重值存储在 pcl::PointXYZINormal 类型的点云数据结构中，并将其添加到 vslam_data 点云中
  }

  auto t1 = std::chrono::system_clock::now();
  long dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  std::cout
      << "landmark ratio \033[34m" << vslam_data->size()
      << "\033[m / \033[34m" << local_landmarks.size()
      << "\033[m" << dt << std::endl;

  return;
}

int BridgeOpenVSLAM::getState() const//getState 函数返回当前 OpenVSLAM 的跟踪状态
{
  return static_cast<int>(SLAM_ptr->get_frame_publisher()->get_tracking_state());
}

cv::Mat BridgeOpenVSLAM::getFrame() const//getFrame 函数返回当前帧的图像数据，调用 OpenVSLAM 的 draw_frame 函数
{
  return SLAM_ptr->get_frame_publisher()->draw_frame(false);
}

Eigen::Matrix4f BridgeOpenVSLAM::getCameraPose() const//getCameraPose 函数返回当前相机的位姿矩阵
{
  return SLAM_ptr->get_map_publisher()->get_current_cam_pose().cast<float>();
}

void BridgeOpenVSLAM::requestReset()//requestReset 函数检查是否已请求重置 SLAM 系统，如果没有，则请求重置
{
  if (!SLAM_ptr->reset_is_requested())
    SLAM_ptr->request_reset();
}

void BridgeOpenVSLAM::execute(const cv::Mat& image)//execute 函数处理输入图像并将其传递给 SLAM 系统。如果跟踪状态变为丢失（Lost），则请求重置 SLAM 系统
{
  SLAM_ptr->feed_monocular_frame(image, 0.05, cv::Mat{});
  if (SLAM_ptr->get_frame_publisher()->get_tracking_state() == openvslam::tracker_state_t::Lost) {
    std::cout << "\n\033[33m ##### Request Reset #####\n\033[m" << std::endl;
    requestReset();
  }
}

void BridgeOpenVSLAM::setCriteria(unsigned int recollection_, float accuracy_)//setCriteria 函数设置地标点的回收和精度标准，并确保它们在合理范围内
{
  recollection = recollection_;
  accuracy = accuracy_;

  recollection = std::max(recollection, 1u);
  accuracy = std::max(accuracy, 0.1f);
  accuracy = std::min(accuracy, 1.0f);
}

std::pair<unsigned int, float> BridgeOpenVSLAM::getCriteria() const
{
  return {recollection, accuracy};
}

}  // namespace iris
