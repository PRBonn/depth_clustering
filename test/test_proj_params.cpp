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

#include "cmake_config.h"
#include "projections/projection_params.h"

using namespace depth_clustering;

TEST(TestProjParams, test_init) {
  ProjectionParams params;
  try {
    params.valid();
    FAIL();
  } catch (const std::runtime_error& error) {
    std::string expected = "Projection parameters invalid.";
    EXPECT_EQ(expected, error.what());
  }
}

TEST(TestProjParams, from_angle) {
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 10_deg, 1_deg),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 10_deg, 1_deg),
                 SpanParams::Direction::HORIZONTAL);
  EXPECT_EQ(true, params.valid());
  EXPECT_EQ(0ul, params.ColFromAngle(0_deg));
  EXPECT_EQ(1ul, params.ColFromAngle(1_deg));
  EXPECT_EQ(2ul, params.ColFromAngle(2_deg));
  EXPECT_EQ(3ul, params.ColFromAngle(3_deg));
  EXPECT_EQ(4ul, params.ColFromAngle(4_deg));
  EXPECT_EQ(5ul, params.ColFromAngle(5_deg));
  EXPECT_EQ(6ul, params.ColFromAngle(6_deg));
  EXPECT_EQ(7ul, params.ColFromAngle(7_deg));
  EXPECT_EQ(8ul, params.ColFromAngle(8_deg));
  EXPECT_EQ(9ul, params.ColFromAngle(9_deg));
  EXPECT_EQ(9ul, params.ColFromAngle(10_deg));

  EXPECT_EQ(0ul, params.RowFromAngle(0_deg));
  EXPECT_EQ(1ul, params.RowFromAngle(1_deg));
  EXPECT_EQ(2ul, params.RowFromAngle(2_deg));
  EXPECT_EQ(3ul, params.RowFromAngle(3_deg));
  EXPECT_EQ(4ul, params.RowFromAngle(4_deg));
  EXPECT_EQ(5ul, params.RowFromAngle(5_deg));
  EXPECT_EQ(6ul, params.RowFromAngle(6_deg));
  EXPECT_EQ(7ul, params.RowFromAngle(7_deg));
  EXPECT_EQ(8ul, params.RowFromAngle(8_deg));
  EXPECT_EQ(9ul, params.RowFromAngle(9_deg));
  EXPECT_EQ(9ul, params.RowFromAngle(10_deg));
}

TEST(TestProjParams, velodyne_16) {
  auto params_ptr = ProjectionParams::VLP_16();
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(16ul, params_ptr->rows());
  EXPECT_EQ(870ul, params_ptr->cols());
}

TEST(TestProjParams, velodyne_64) {
  auto params_ptr = ProjectionParams::HDL_64();
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(64ul, params_ptr->rows());
  EXPECT_EQ(870ul, params_ptr->cols());
}

TEST(TestProjParams, velodyne_32) {
  auto params_ptr = ProjectionParams::HDL_32();
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(32ul, params_ptr->rows());
  EXPECT_EQ(870ul, params_ptr->cols());
}

TEST(TestProjParams, velodyne_64_equal) {
  auto params_ptr = ProjectionParams::HDL_64_EQUAL();
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(64ul, params_ptr->rows());
  EXPECT_EQ(870ul, params_ptr->cols());
}


TEST(TestProjParams, full_sphere) {
  auto params_ptr = ProjectionParams::FullSphere(1_deg);
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(180ul, params_ptr->rows());
  EXPECT_EQ(360ul, params_ptr->cols());
}

