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

#include "./utils.h"

#include <projections/projection_params.h>
#include <utils/folder_reader.h>
#include <utils/velodyne_utils.h>

#if PCL_FOUND
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#endif  // PCL_FOUND

using depth_clustering::Cloud;
using depth_clustering::MatFromDepthPng;
using depth_clustering::ProjectionParams;
using depth_clustering::ReadKittiCloud;
using depth_clustering::ReadKittiCloudTxt;

QString appendPaths(const QString &path1, const QString &path2) {
  return QDir::cleanPath(path1 + QDir::separator() + path2);
}

QImage MatToQImage(const cv::Mat &image) {
  auto qimage = QImage(image.cols, image.rows, QImage::Format_RGB888);
  if (image.type() == CV_32F) {
    for (int r = 0; r < image.rows; ++r) {
      for (int c = 0; c < image.cols; ++c) {
        if (image.at<float>(r, c) == 666) {
          auto color = qRgb(0, 200, 0);
          qimage.setPixel(c, r, color);
          continue;
        }
        const float &val = image.at<float>(r, c) * 10;
        auto color = qRgb(val, val, val);
        qimage.setPixel(c, r, color);
      }
    }
  } else {
    for (int r = 0; r < image.rows; ++r) {
      for (int c = 0; c < image.cols; ++c) {
        auto val = image.at<cv::Vec3b>(r, c);
        auto color = qRgb(val[0], val[1], val[2]);
        qimage.setPixel(c, r, color);
      }
    }
  }
  return qimage;
}

Cloud::Ptr CloudFromFile(const std::string &file_name,
                         const ProjectionParams &proj_params) {
  QFileInfo fi(QString::fromStdString(file_name));
  QString name = fi.fileName();
  Cloud::Ptr cloud = nullptr;
  if (name.endsWith(".pcd")) {
#if PCL_FOUND
    pcl::PointCloud<pcl::PointXYZL> pcl_cloud;
    pcl::io::loadPCDFile(file_name, pcl_cloud);
    cloud = Cloud::FromPcl<pcl::PointXYZL>(pcl_cloud);
    cloud->InitProjection(proj_params);
#endif  // PCL_FOUND
  } else if (name.endsWith(".png") || name.endsWith(".exr")) {
    cloud = Cloud::FromImage(MatFromDepthPng(file_name), proj_params);
  } else if (name.endsWith(".txt")) {
    cloud = ReadKittiCloudTxt(file_name);
    cloud->InitProjection(proj_params);
  } else if (name.endsWith(".bin")) {
    cloud = ReadKittiCloud(file_name);
    cloud->InitProjection(proj_params);
  }
  // if (cloud) {
  //   // if the cloud was actually set, then set the name in the gui
  //   ui->lbl_cloud_name->setText(name);
  // }
  return cloud;
}
