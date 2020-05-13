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
#include <Eigen/Core>

#include <limits>

#include "utils/rich_point.h"

using Eigen::Vector3f;
using namespace depth_clustering;

TEST(RichPointTest, Init) {
  RichPoint point;
  auto eps = std::numeric_limits<float>::epsilon();
  EXPECT_NEAR(0.0, point.x(), eps);
  EXPECT_NEAR(0.0, point.y(), eps);
  EXPECT_NEAR(0.0, point.z(), eps);
  EXPECT_NEAR(0, point.ring(), eps);
}

TEST(RichPointTest, InitFull) {
  RichPoint point(1, 2, 3, 4);
  auto eps = std::numeric_limits<float>::epsilon();
  EXPECT_NEAR(1.0, point.x(), eps);
  EXPECT_NEAR(2.0, point.y(), eps);
  EXPECT_NEAR(3.0, point.z(), eps);
  EXPECT_NEAR(4, point.ring(), eps);
}

TEST(RichPointTest, InitPartial) {
  RichPoint point(1, 2, 3);
  auto eps = std::numeric_limits<float>::epsilon();
  EXPECT_NEAR(1.0, point.x(), eps);
  EXPECT_NEAR(2.0, point.y(), eps);
  EXPECT_NEAR(3.0, point.z(), eps);
  EXPECT_NEAR(0, point.ring(), eps);
}

TEST(RichPointTest, InitEigen) {
  Vector3f vec(1, 2, 3);
  RichPoint point(vec);
  auto eps = std::numeric_limits<float>::epsilon();
  EXPECT_NEAR(1.0, point.x(), eps);
  EXPECT_NEAR(2.0, point.y(), eps);
  EXPECT_NEAR(3.0, point.z(), eps);
  EXPECT_NEAR(0, point.ring(), eps);
}

TEST(RichPointTest, Dist2D) {
  Vector3f vec(1, 1, 1);
  RichPoint point(vec);
  auto eps = std::numeric_limits<float>::epsilon();
  EXPECT_NEAR(sqrt(2), point.DistToSensor2D(), eps);
}

TEST(RichPointTest, Dist3D) {
  Vector3f vec(1, 1, 1);
  RichPoint point(vec);
  auto eps = std::numeric_limits<float>::epsilon();
  EXPECT_NEAR(sqrt(3), point.DistToSensor3D(), eps);
}

TEST(RichPointTest, Assign) {
  Vector3f vec(1, 1, 1);
  RichPoint point(vec);
  point.ring() = 20;
  RichPoint point2;
  point2 = point;
  auto eps = std::numeric_limits<float>::epsilon();
  EXPECT_NEAR(point.x(), point2.x(), eps);
  EXPECT_NEAR(point.y(), point2.y(), eps);
  EXPECT_NEAR(point.z(), point2.z(), eps);
  EXPECT_NEAR(point.ring(), point2.ring(), eps);
}

TEST(RichPointTest, Equals) {
  Vector3f vec(1, 1, 1);
  RichPoint point(vec);
  RichPoint point2(vec);
  EXPECT_TRUE(point == point2);
  point.ring() = 20;
  EXPECT_FALSE(point == point2);
}
