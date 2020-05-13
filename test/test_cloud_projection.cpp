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

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include <string>

#include "projections/cloud_projection.h"
#include "projections/spherical_projection.h"
#include "utils/cloud.h"
#include "utils/radians.h"

using cv::Mat;

using namespace depth_clustering;

using Dir = depth_clustering::SpanParams::Direction;

TEST(CloudProjectionTest, SingleLine) {
  Radians horiz_span = 10_deg;
  Radians horiz_step = 1_deg;
  float x_dist = 10.0f;
  auto cloud_ptr = Cloud::Ptr(new Cloud);
  auto start_angle = -horiz_span / 2;
  for (int i = 0; i < 10; ++i) {
    Radians angle = start_angle + horiz_step * i;
    fprintf(stderr, "adding angle: %f\n", angle.ToDegrees());
    cloud_ptr->push_back(
        RichPoint(x_dist, x_dist * tan(angle.val()) + 0.01, 0));
  }

  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg, 1_deg), Dir::VERTICAL);
  params.SetSpan(SpanParams(-horiz_span / 2, horiz_span / 2, horiz_step),
                 Dir::HORIZONTAL);
  cloud_ptr->InitProjection(params);
  auto projection = cloud_ptr->projection_ptr();

  EXPECT_EQ(1ul, projection->rows());
  EXPECT_EQ(10ul, projection->cols());
  EXPECT_EQ(0ul, projection->at(0, 0).points().front());
  EXPECT_EQ(1ul, projection->at(0, 1).points().front());
  EXPECT_EQ(2ul, projection->at(0, 2).points().front());
  EXPECT_EQ(3ul, projection->at(0, 3).points().front());
  EXPECT_EQ(4ul, projection->at(0, 4).points().front());
  EXPECT_EQ(5ul, projection->at(0, 5).points().front());
  EXPECT_EQ(6ul, projection->at(0, 6).points().front());
  EXPECT_EQ(7ul, projection->at(0, 7).points().front());
  EXPECT_EQ(8ul, projection->at(0, 8).points().front());
  EXPECT_EQ(9ul, projection->at(0, 9).points().front());
}

TEST(CloudProjectionTest, PlanePatch) {
  Radians horizontal_span = 5_deg;
  Radians vertical_span = 5_deg;
  Radians step = 1_deg;
  float x_dist = 10.0f;
  auto cloud_ptr = Cloud::Ptr(new Cloud);
  float start_horizontal_angle = -horizontal_span.val() / 2;
  float start_vertical_angle = -vertical_span.val() / 2;
  float step_angle = step.val();
  for (int v = 0; v < 5; ++v) {
    for (int h = 0; h < 5; ++h) {
      cloud_ptr->push_back(RichPoint(
          x_dist, x_dist * tan(start_horizontal_angle + step_angle * h) + 0.01,
          x_dist * tan(start_vertical_angle + step_angle * v) + 0.01));
    }
  }

  ProjectionParams params;
  params.SetSpan(SpanParams(-vertical_span / 2, vertical_span / 2, step),
                 Dir::VERTICAL);
  params.SetSpan(SpanParams(-horizontal_span / 2, horizontal_span / 2, step),
                 Dir::HORIZONTAL);
  SphericalProjection storage(params);
  storage.InitFromPoints(cloud_ptr->points());

  EXPECT_EQ(5ul, storage.rows());
  EXPECT_EQ(5ul, storage.cols());
  EXPECT_EQ(0ul, storage.at(0, 0).points().front());
  EXPECT_EQ(1ul, storage.at(0, 1).points().front());
  EXPECT_EQ(2ul, storage.at(0, 2).points().front());
  EXPECT_EQ(3ul, storage.at(0, 3).points().front());
  EXPECT_EQ(4ul, storage.at(0, 4).points().front());

  EXPECT_EQ(5ul, storage.at(1, 0).points().front());
  EXPECT_EQ(6ul, storage.at(1, 1).points().front());
  EXPECT_EQ(7ul, storage.at(1, 2).points().front());
  EXPECT_EQ(8ul, storage.at(1, 3).points().front());
  EXPECT_EQ(9ul, storage.at(1, 4).points().front());

  EXPECT_EQ(10ul, storage.at(2, 0).points().front());
  EXPECT_EQ(11ul, storage.at(2, 1).points().front());
  EXPECT_EQ(12ul, storage.at(2, 2).points().front());
  EXPECT_EQ(13ul, storage.at(2, 3).points().front());
  EXPECT_EQ(14ul, storage.at(2, 4).points().front());

  EXPECT_EQ(15ul, storage.at(3, 0).points().front());
  EXPECT_EQ(16ul, storage.at(3, 1).points().front());
  EXPECT_EQ(17ul, storage.at(3, 2).points().front());
  EXPECT_EQ(18ul, storage.at(3, 3).points().front());
  EXPECT_EQ(19ul, storage.at(3, 4).points().front());

  EXPECT_EQ(20ul, storage.at(4, 0).points().front());
  EXPECT_EQ(21ul, storage.at(4, 1).points().front());
  EXPECT_EQ(22ul, storage.at(4, 2).points().front());
  EXPECT_EQ(23ul, storage.at(4, 3).points().front());
  EXPECT_EQ(24ul, storage.at(4, 4).points().front());
}

TEST(CloudProjectionTest, Circle) {
  Radians horiz_span = 360_deg;
  Radians horiz_step = 30_deg;
  float distance = 10.0f;
  auto cloud_ptr = Cloud::Ptr(new Cloud);
  auto start_angle = -horiz_span / 2;
  for (int i = 0; i < 12; ++i) {
    auto angle = start_angle + horiz_step * i;
    fprintf(stderr, "adding angle: %f deg\n", angle.ToDegrees());
    cloud_ptr->push_back(RichPoint(distance * cos(angle.val() + 0.001),
                                   distance * sin(angle.val() + 0.001), 0));
  }

  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg, 1_deg), Dir::VERTICAL);
  params.SetSpan(SpanParams(-horiz_span / 2, horiz_span / 2, horiz_step),
                 Dir::HORIZONTAL);
  SphericalProjection storage(params);
  storage.InitFromPoints(cloud_ptr->points());

  EXPECT_EQ(1ul, storage.rows());
  EXPECT_EQ(12ul, storage.cols());
  EXPECT_EQ(0ul, storage.at(0, 0).points().front());
  EXPECT_EQ(1ul, storage.at(0, 1).points().front());
  EXPECT_EQ(2ul, storage.at(0, 2).points().front());
  EXPECT_EQ(3ul, storage.at(0, 3).points().front());
  EXPECT_EQ(4ul, storage.at(0, 4).points().front());
  EXPECT_EQ(5ul, storage.at(0, 5).points().front());
  EXPECT_EQ(6ul, storage.at(0, 6).points().front());
  EXPECT_EQ(7ul, storage.at(0, 7).points().front());
  EXPECT_EQ(8ul, storage.at(0, 8).points().front());
  EXPECT_EQ(9ul, storage.at(0, 9).points().front());
}

TEST(CloudProjectionTest, BigCylinder) {
  Radians horizontal_span = 360_deg;
  Radians vertical_span = 24_deg;
  Radians hor_step = 30_deg;
  Radians ver_step = 2_deg;
  float x_dist = 10.0f;
  auto cloud_ptr = Cloud::Ptr(new Cloud);
  float start_vertical_angle = -vertical_span.val() / 2;
  float start_horizontal_angle = -horizontal_span.val() / 2;
  float hor_step_angle = hor_step.val();
  float ver_step_angle = ver_step.val();
  for (int v = 0; v < 12; ++v) {
    float ver_angle = start_vertical_angle + ver_step_angle * v;
    for (int h = 0; h < 12; ++h) {
      float hor_angle = start_horizontal_angle + hor_step_angle * h + 0.001;
      cloud_ptr->push_back(RichPoint(x_dist * cos(hor_angle),
                                     x_dist * sin(hor_angle),
                                     x_dist * tan(ver_angle) + 0.001));
    }
  }

  ProjectionParams params;
  params.SetSpan(SpanParams(-vertical_span / 2, vertical_span / 2, ver_step),
                 Dir::VERTICAL);
  params.SetSpan(
      SpanParams(-horizontal_span / 2, horizontal_span / 2, hor_step),
      Dir::HORIZONTAL);
  SphericalProjection storage(params);
  storage.InitFromPoints(cloud_ptr->points());

  EXPECT_EQ(12ul, storage.rows());
  EXPECT_EQ(12ul, storage.cols());

  // first 5 points
  EXPECT_EQ(0ul, storage.at(0, 0).points().front());
  EXPECT_EQ(1ul, storage.at(0, 1).points().front());
  EXPECT_EQ(2ul, storage.at(0, 2).points().front());
  EXPECT_EQ(3ul, storage.at(0, 3).points().front());
  EXPECT_EQ(4ul, storage.at(0, 4).points().front());
}

TEST(CloudProjectionTest, FullSphere) {
  Radians horizontal_span = 360_deg;
  Radians vertical_span = 180_deg;
  Radians hor_step = 30_deg;
  Radians ver_step = 15_deg;
  float radius = 100.0f;
  auto cloud_ptr = Cloud::Ptr(new Cloud);
  float start_vertical_angle = -vertical_span.val() / 2;
  float start_horizontal_angle = -horizontal_span.val() / 2;
  float hor_step_angle = hor_step.val();
  float ver_step_angle = ver_step.val();
  for (int v = 0; v < 12; ++v) {
    float ver_angle = start_vertical_angle + ver_step_angle * v + 0.01f;
    for (int h = 0; h < 12; ++h) {
      float hor_angle = start_horizontal_angle + hor_step_angle * h + 0.01f;
      cloud_ptr->push_back(RichPoint(radius * cos(hor_angle) * cos(ver_angle),
                                     radius * sin(hor_angle) * cos(ver_angle),
                                     radius * sin(ver_angle) + 0.001));
    }
  }

  ProjectionParams params;
  params.SetSpan(SpanParams(-vertical_span / 2, vertical_span / 2, ver_step),
                 Dir::VERTICAL);
  params.SetSpan(
      SpanParams(-horizontal_span / 2, horizontal_span / 2, hor_step),
      Dir::HORIZONTAL);
  SphericalProjection storage(params);
  storage.InitFromPoints(cloud_ptr->points());
  Mat image = storage.depth_image();
  cv::imwrite("sphere.png", image);

  EXPECT_EQ(12ul, storage.rows());
  EXPECT_EQ(12ul, storage.cols());

  // first 5 points
  EXPECT_EQ(0ul, storage.at(0, 0).points().front());
  EXPECT_EQ(1ul, storage.at(0, 1).points().front());
  EXPECT_EQ(2ul, storage.at(0, 2).points().front());
  EXPECT_EQ(3ul, storage.at(0, 3).points().front());
  EXPECT_EQ(4ul, storage.at(0, 4).points().front());

  float eps = 0.001;
  EXPECT_NEAR(image.at<float>(0, 0), radius, eps);
  EXPECT_NEAR(image.at<float>(image.rows - 1, image.cols - 1), radius, eps);
}

TEST(CloudProjectionTest, WrongStorage) {
  ProjectionParams params;
  try {
    SphericalProjection storage(params);
  } catch (std::runtime_error& e) {
    std::string error_msg = "Projection parameters invalid.";
    EXPECT_EQ(error_msg, e.what());
  }
}

TEST(CloudProjectionTest, WrongCloud) {
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg, 1_deg), Dir::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg, 1_deg), Dir::HORIZONTAL);
  SphericalProjection storage(params);
  Cloud cloud;
  try {
    storage.InitFromPoints(cloud.points());
  } catch (std::runtime_error& e) {
    std::string error_msg = "cannot fill from cloud: no points";
    EXPECT_EQ(error_msg, e.what());
  }
}

TEST(CloudProjectionTest, FromDepthImage) {
  Radians span = 10_deg;
  Radians step = 1_deg;
  float dist = 10.0f;
  cv::Mat image = cv::Mat::ones(10, 10, CV_32F) * dist;

  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, span, step), Dir::VERTICAL);
  params.SetSpan(SpanParams(0_deg, span, step), Dir::HORIZONTAL);
  auto cloud = Cloud::FromImage(image, params);
  auto projection = cloud->projection_ptr();
  cv::Mat image_res = projection->depth_image();
  EXPECT_EQ(10ul, projection->rows());
  EXPECT_EQ(10ul, projection->cols());
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(image.at<float>(i, 0), image_res.at<float>(i, 0));
    EXPECT_EQ(image.at<float>(0, i), image_res.at<float>(0, i));
    EXPECT_EQ(image.at<float>(i, i), image_res.at<float>(i, i));
  }
}
