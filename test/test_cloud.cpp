// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include <gtest/gtest.h>
#include <Eigen/Core>

#include <limits>

#include "utils/cloud.h"
#include "utils/pose.h"
#include "utils/rich_point.h"

using Eigen::Vector3f;

using std::sin;
using std::cos;

using namespace depth_clustering;

TEST(CloudTest, InitEmpty) {
  Cloud cloud;
  double eps = 0.000001;
  EXPECT_NEAR(0.0, cloud.pose().x(), eps);
  EXPECT_NEAR(0.0, cloud.pose().y(), eps);
  EXPECT_NEAR(0.0, cloud.pose().theta(), eps);
  EXPECT_EQ(0, cloud.size());
}

TEST(CloudTest, InitPose) {
  double eps = 0.000001;
  Eigen::Affine3f m;
  m.matrix() << cos(M_PI / 3), sin(M_PI / 3), 0, 1, sin(M_PI / 3),
      cos(M_PI / 3), 0, 2, 0, 0, 1, 0, 0, 0, 0, 1;
  Pose pose(m);
  Cloud cloud(pose);
  EXPECT_NEAR(1, cloud.pose().x(), eps);
  EXPECT_NEAR(2, cloud.pose().y(), eps);
  EXPECT_NEAR(M_PI / 3, cloud.pose().theta(), eps);
  EXPECT_EQ(0, cloud.size());
}

TEST(CloudTest, InitCloudWithPose) {
  double eps = 0.000001;
  Eigen::Affine3f m;
  m.matrix() << cos(M_PI / 3), sin(M_PI / 3), 0, 1, sin(M_PI / 3),
      cos(M_PI / 3), 0, 2, 0, 0, 1, 0, 0, 0, 0, 1;
  Cloud cloud;
  cloud.push_back(RichPoint(1, 2, 3));
  cloud.pose() = Pose(m);
  EXPECT_NEAR(1, cloud.pose().x(), eps);
  EXPECT_NEAR(2, cloud.pose().y(), eps);
  EXPECT_NEAR(M_PI / 3, cloud.pose().theta(), eps);
  EXPECT_EQ(1, cloud.size());
}

TEST(CloudTest, CloudCopy) {
  double eps = 0.00001;
  Cloud cloud;
  cloud.push_back(RichPoint(1, 2, 3));
  cloud.pose() = Pose(1, 1, 1);
  cloud.sensor_pose() = Pose(2, 2, 2);
  auto cloud_copy(cloud);
  EXPECT_NEAR(1, cloud_copy.pose().x(), eps);
  EXPECT_NEAR(1, cloud_copy.pose().y(), eps);
  EXPECT_NEAR(1, cloud_copy.pose().theta(), eps);
  EXPECT_NEAR(2, cloud_copy.sensor_pose().x(), eps);
  EXPECT_NEAR(2, cloud_copy.sensor_pose().y(), eps);
  EXPECT_NEAR(2, cloud_copy.sensor_pose().theta(), eps);
  EXPECT_NEAR(1, cloud_copy[0].x(), eps);
  EXPECT_NEAR(2, cloud_copy[0].y(), eps);
  EXPECT_NEAR(3, cloud_copy[0].z(), eps);
  EXPECT_EQ(1, cloud.size());
}

TEST(CloudTest, EmptyCloudProjectionPixels) {
  Cloud cloud;
  auto points = cloud.PointsProjectedToPixel(10, 10);
  ASSERT_TRUE(points.empty());
}

TEST(CloudTest, TransformInPlace) {
  float eps = std::numeric_limits<float>::epsilon();
  Cloud cloud;
  cloud.push_back(RichPoint(1, 1, 1));

  Pose transform = Pose(1, 0, 0);
  cloud.TransformInPlace(transform);
  ASSERT_NEAR(cloud[0].x(), 2, eps);
  ASSERT_NEAR(cloud[0].y(), 1, eps);
  ASSERT_NEAR(cloud[0].z(), 1, eps);
}

TEST(CloudTest, Transform) {
  float eps = std::numeric_limits<float>::epsilon();
  Cloud cloud;
  cloud.push_back(RichPoint(1, 1, 1));

  Pose transform = Pose(1, 0, 0);
  auto cloud_ptr = cloud.Transform(transform);
  ASSERT_NEAR(cloud_ptr->at(0).x(), 2, eps);
  ASSERT_NEAR(cloud_ptr->at(0).y(), 1, eps);
  ASSERT_NEAR(cloud_ptr->at(0).z(), 1, eps);
}

TEST(CloudTest, InitProjectionTwice) {
  using Dir = depth_clustering::SpanParams::Direction;

  Radians horizontal_span = 360_deg;
  Radians vertical_span = 180_deg;
  Radians hor_step = 30_deg;
  Radians ver_step = 15_deg;
  float radius = 1.0f;
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
  cloud_ptr->InitProjection(params);
  try {
    cloud_ptr->InitProjection(params);
    FAIL();
  } catch (const std::runtime_error& e) {
    std::string error_msg = "projection is already initialized";
    ASSERT_EQ(e.what(), error_msg);
  }
}

TEST(CloudTest, ProjectionFullSphere) {
  using Dir = depth_clustering::SpanParams::Direction;

  Radians horizontal_span = 360_deg;
  Radians vertical_span = 180_deg;
  Radians hor_step = 30_deg;
  Radians ver_step = 15_deg;
  float radius = 1.0f;
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

  cloud_ptr->InitProjection(params);
  EXPECT_TRUE(cloud_ptr->projection_ptr() != nullptr);

  auto cloud_unproject_ptr =
      Cloud::FromImage(cloud_ptr->projection_ptr()->depth_image(), params);

  EXPECT_EQ(cloud_unproject_ptr->size(), cloud_ptr->size());
  auto ptr1 = cloud_unproject_ptr->PointsProjectedToPixel(2, 2).front();
  auto ptr2 = cloud_ptr->PointsProjectedToPixel(2, 2).front();
  EXPECT_TRUE(ptr1 != nullptr);
  EXPECT_TRUE(ptr2 != nullptr);
  // give some error margin as the backprojected cloud will put point in the
  // center of a cell.
  EXPECT_NEAR(ptr1->x(), ptr2->x(), 0.05);
  EXPECT_NEAR(ptr1->y(), ptr2->y(), 0.05);
  EXPECT_NEAR(ptr1->z(), ptr2->z(), 0.05);
}
