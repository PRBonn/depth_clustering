// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "visualization/cloud_saver.h"

#include <string>

namespace depth_clustering {

void VectorCloudSaver::OnNewObjectReceived(
    const std::unordered_map<uint16_t, Cloud>& clouds, const int) {
  if (_folder_counter++ % _save_every > 0) {
    // nope, skip this one
    return;
  }
  auto folder_name = _prefix + "_" + WithLeadingZerosStr(_folder_counter - 1);
  boost::filesystem::path dir(folder_name);
  fprintf(stderr, "INFO: saving clusters to '%s'\n", folder_name.c_str());

  if (boost::filesystem::create_directory(dir)) {
    size_t cloud_counter = 0;
    for (const auto& kv : clouds) {
      const auto& cloud = kv.second;
      auto cloud_name = dir.string() + "/cloud_" +
                        WithLeadingZerosStr(cloud_counter++) + ".pcd";
      pcl::io::savePCDFileBinary(cloud_name, *(cloud.ToPcl()));
    }
  }
}

std::string VectorCloudSaver::WithLeadingZerosStr(int num) {
  size_t leading_zeros_num = 6;
  auto counter_str = std::to_string(num);
  return std::string(leading_zeros_num - counter_str.size(), '0')
      .append(counter_str);
}

void DepthMapSaver::OnNewObjectReceived(
    const std::unordered_map<uint16_t, cv::Mat>& clusters, const int) {
  if (_folder_counter++ % _save_every > 0) {
    // nope, skip this one
    return;
  }
  auto folder_name = _prefix + "_" + WithLeadingZerosStr(_folder_counter - 1);
  boost::filesystem::path dir(folder_name);
  fprintf(stderr, "INFO: saving clusters to '%s'\n", folder_name.c_str());

  if (boost::filesystem::create_directory(dir)) {
    size_t cloud_counter = 0;
    for (const auto& cluster : clusters) {
      auto image_name = dir.string() + "/depth_cluster_" +
                        WithLeadingZerosStr(cloud_counter++) + ".exr";
      cv::imwrite(image_name, cluster.second);
    }
  }
}

std::string DepthMapSaver::WithLeadingZerosStr(int num) {
  size_t leading_zeros_num = 6;
  auto counter_str = std::to_string(num);
  return std::string(leading_zeros_num - counter_str.size(), '0')
      .append(counter_str);
}

void CloudSaver::OnNewObjectReceived(const Cloud& cloud, const int) {
  pcl::io::savePCDFileBinary(
      _prefix + "_" + std::to_string(_counter++) + ".pcd", *(cloud.ToPcl()));
}

}  // namespace depth_clustering
