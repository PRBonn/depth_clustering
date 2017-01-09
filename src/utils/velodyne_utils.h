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

#ifndef SRC_UTILS_VELODYNE_UTILS_H_
#define SRC_UTILS_VELODYNE_UTILS_H_

#include <opencv2/opencv.hpp>

#include <string>

#include "utils/cloud.h"
#include "projections/projection_params.h"

namespace depth_clustering {

Cloud::Ptr CloudFromMat(const cv::Mat& image, const ProjectionParams& config);
Cloud::Ptr ReadKittiCloud(const std::string& path);
Cloud::Ptr ReadKittiCloudTxt(const std::string& path);
cv::Mat MatFromDepthPng(const std::string& path);

}  // namespace depth_clustering

#endif  // SRC_UTILS_VELODYNE_UTILS_H_
