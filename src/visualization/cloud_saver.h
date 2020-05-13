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
