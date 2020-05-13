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

#include "utils/velodyne_utils.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cassert>
#include <fstream>
#include <string>
#include <vector>

#include "utils/cloud.h"

namespace depth_clustering {

using std::string;
using std::vector;

Cloud::Ptr ReadKittiCloudTxt(const string& path) {
  std::locale::global(std::locale("en_US.UTF-8"));
  fprintf(stderr, "Reading cloud from %s.\n", path.c_str());
  auto cloud_ptr = Cloud::Ptr(new Cloud);
  std::ifstream file(path.c_str());
  for (std::string line; std::getline(file, line, '\n');) {
    // here we parse the line
    vector<string> coords_str;
    boost::split(coords_str, line, boost::is_any_of(" "));
    if (coords_str.size() != 4) {
      fprintf(stderr, "ERROR: format of line is wrong.\n");
      continue;
    }
    RichPoint point;
    point.x() = std::stof(coords_str[0]);
    point.y() = std::stof(coords_str[1]);
    point.z() = std::stof(coords_str[2]);
    cloud_ptr->push_back(point);
  }
  return cloud_ptr;
}

Cloud::Ptr CloudFromMat(const cv::Mat& image, const ProjectionParams& config) {
  auto cloud_ptr = Cloud::Ptr(new Cloud);
  for (int row = 0; row < image.rows; ++row) {
    for (int col = 0; col < image.cols; ++col) {
      float meters = image.at<float>(row, col);
      float angle_xy = config.AngleFromCol(col).val();
      float angle_z = config.AngleFromRow(row).val();
      RichPoint point;
      point.x() = meters * cos(angle_z) * cos(angle_xy);
      point.y() = meters * cos(angle_z) * sin(angle_xy);
      point.z() = meters * sin(angle_z);
      point.ring() = row;
      cloud_ptr->push_back(point);
    }
  }
  return cloud_ptr;
}

Cloud::Ptr ReadKittiCloud(const string& path) {
  Cloud::Ptr cloud(new Cloud);
  std::fstream file(path.c_str(), std::ios::in | std::ios::binary);
  if (file.good()) {
    file.seekg(0, std::ios::beg);
    float intensity = 0;
    for (int i = 0; file.good() && !file.eof(); ++i) {
      RichPoint point;
      file.read(reinterpret_cast<char*>(&point.x()), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.y()), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.z()), sizeof(float));
      // ignore intensity
      file.read(reinterpret_cast<char*>(&intensity), sizeof(float));
      cloud->push_back(point);
    }
    file.close();
  }
  return cloud;
}

cv::Mat FixKITTIDepth(const cv::Mat& original) {
  cv::Mat fixed = original;
  for (int r = 0; r < fixed.rows; ++r) {
    auto correction = MOOSMAN_CORRECTIONS[r];
    for (int c = 0; c < fixed.cols; ++c) {
      if (fixed.at<float>(r, c) < 0.001f) {
        continue;
      }
      fixed.at<float>(r, c) -= correction;
    }
  }
  return fixed;
}

cv::Mat MatFromDepthPng(const string& path) {
  cv::Mat depth_image = cv::imread(path, CV_LOAD_IMAGE_ANYDEPTH);
  depth_image.convertTo(depth_image, CV_32F);
  depth_image /= 500.;
  return FixKITTIDepth(depth_image);
}

}  // namespace depth_clustering
