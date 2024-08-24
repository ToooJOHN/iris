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

#pragma once
#include <sstream>

namespace iris
{
namespace map
{
struct Parameter {
  Parameter(
      const std::string& pcd_file,
      float voxel_grid_leaf,
      float normal_search_radius,
      float submap_grid_leaf)
      : pcd_file(pcd_file),
        voxel_grid_leaf(voxel_grid_leaf),
        normal_search_radius(normal_search_radius),
        submap_grid_leaf(submap_grid_leaf) {}

  std::string pcd_file;
  float voxel_grid_leaf;
  float normal_search_radius;
  float submap_grid_leaf;

  std::string toString() const
  {
    std::stringstream ss;
    ss << pcd_file << " " << std::to_string(voxel_grid_leaf) << " " << std::to_string(normal_search_radius);
    //这行代码将 pcd_file 的值、voxel_grid_leaf 的值（以字符串形式）和 normal_search_radius 的值（以字符串形式）依次插入到 ss 中，并用空格分隔。
    //比如，如果 pcd_file 是 "example.pcd"，voxel_grid_leaf 是 0.05f，normal_search_radius 是 0.1f，那么最终构建的字符串将会是："example.pcd 0.050000 0.100000"

    return ss.str();
  }
};

}  // namespace map
}  // namespace iris