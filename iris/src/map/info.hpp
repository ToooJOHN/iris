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

#pragma once  // 保證該頭文件在一次編譯過程中只會被編譯一次，防止重複包含
#include <Eigen/Dense>  // 包含 Eigen 庫，用於線性代數操作，如矩陣和向量計算
#include <limits>  // 包含數值限制的標準庫，例如浮點數的最大值、最小值和 NaN 值

namespace iris  // 定義命名空間 iris，防止命名衝突
{
namespace map  // 定義子命名空間 map，與地圖相關的代碼組織在一起
{

struct Info {  // 定義一個名為 Info 的結構體，用於存儲位置信息
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 保證對齊方式適合 Eigen 庫的內存要求，避免內存錯誤

  // 成員變量，初始化為非數值（NaN），表示無效或未初始化的狀態
  float x = std::numeric_limits<float>::quiet_NaN();  
  float y = std::numeric_limits<float>::quiet_NaN();  
  float theta = std::numeric_limits<float>::quiet_NaN();  // 方向角，通常以弧度表示

  Info() {}  // 默認構造函數，不做任何操作

  // 帶參構造函數，初始化 x, y 和 theta 成員變量
  Info(float x, float y, float theta) : x(x), y(y), theta(theta) {}

  // 返回包含 x 和 y 坐標的 Eigen 二維向量
  Eigen::Vector2f xy() const { return Eigen::Vector2f(x, y); }

  // 將 x, y, theta 轉換為字符串並返回，用於打印或調試
  std::string toString() const
  {
    return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(theta);
  }

  // 比較兩個 Info 對象是否相等，允許在浮點數比較時有極小誤差
  bool isEqual(const Info& a, const Info& b) const
  {
    constexpr float EPSILON = 1e-7f;  // 定義浮點數比較的容差範圍
    if (std::fabs(a.x - b.x) > EPSILON)  // 比較 x 坐標
      return false;
    if (std::fabs(a.y - b.y) > EPSILON)  // 比較 y 坐標
      return false;
    if (std::fabs(a.theta - b.theta) > EPSILON)  // 比較 theta 方向角
      return false;
    return true;  // 當所有比較都在容差範圍內時，返回 true
  }

  // 重載相等運算符，用於直接比較兩個 Info 對象
  bool operator==(const Info& other) const
  {
    return isEqual(*this, other);  // 調用 isEqual 方法進行比較
  }

  // 重載不相等運算符，用於直接比較兩個 Info 對象
  bool operator!=(const Info& other) const
  {
    return !isEqual(*this, other);  // 調用 isEqual 方法進行比較，並取反
  }
};

}  // namespace map
}  // namespace iris
