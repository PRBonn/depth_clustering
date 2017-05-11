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

#ifndef SRC_VISUALIZATION_CLOUD_SAVER_H_
#define SRC_VISUALIZATION_CLOUD_SAVER_H_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "communication/abstract_client.h"
#include "utils/cloud.h"

namespace depth_clustering {

class VectorCloudSaver
    : public AbstractClient<std::unordered_map<uint16_t, Cloud>> {
 public:
  explicit VectorCloudSaver(const std::string& prefix,
                            const size_t save_every = 1)
      : _prefix(prefix), _save_every(save_every), _folder_counter(0) {}
  void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds,
                           const int id) override;
  virtual ~VectorCloudSaver() {}

 private:
  static std::string WithLeadingZerosStr(int num);

  std::string _prefix;
  size_t _save_every;
  size_t _folder_counter;
};

class DepthMapSaver
    : public AbstractClient<std::unordered_map<uint16_t, cv::Mat>> {
 public:
  explicit DepthMapSaver(const std::string& prefix, const size_t save_every = 1)
      : _prefix(prefix), _save_every(save_every), _folder_counter(0) {}

  void OnNewObjectReceived(
      const std::unordered_map<uint16_t, cv::Mat>& clusters,
      const int id) override;
  virtual ~DepthMapSaver() {}

 private:
  static std::string WithLeadingZerosStr(int num);

  std::string _prefix;
  size_t _save_every;
  size_t _folder_counter;
};

class CloudSaver : public AbstractClient<Cloud> {
 public:
  explicit CloudSaver(const std::string& prefix)
      : _prefix(prefix), _counter(0) {}

  void OnNewObjectReceived(const Cloud& cloud, const int id) override;
  virtual ~CloudSaver() {}

 private:
  std::string _prefix;
  size_t _counter;
};

}  // namespace depth_clustering

#endif  // SRC_VISUALIZATION_CLOUD_SAVER_H_
