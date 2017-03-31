// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "visualization/cloud_saver.h"

#include <string>

namespace depth_clustering {

void VectorCloudSaver::OnNewObjectReceived(
    const std::unordered_map<uint16_t, Cloud>& clouds, const int id) {
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
    const std::unordered_map<uint16_t, cv::Mat>& clusters, const int id) {
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

void CloudSaver::OnNewObjectReceived(const Cloud& cloud, const int id) {
  pcl::io::savePCDFileBinary(
      _prefix + "_" + std::to_string(_counter++) + ".pcd", *(cloud.ToPcl()));
}

}  // namespace depth_clustering
