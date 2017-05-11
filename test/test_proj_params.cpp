// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

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
  EXPECT_EQ(0, params.ColFromAngle(0_deg));
  EXPECT_EQ(1, params.ColFromAngle(1_deg));
  EXPECT_EQ(2, params.ColFromAngle(2_deg));
  EXPECT_EQ(3, params.ColFromAngle(3_deg));
  EXPECT_EQ(4, params.ColFromAngle(4_deg));
  EXPECT_EQ(5, params.ColFromAngle(5_deg));
  EXPECT_EQ(6, params.ColFromAngle(6_deg));
  EXPECT_EQ(7, params.ColFromAngle(7_deg));
  EXPECT_EQ(8, params.ColFromAngle(8_deg));
  EXPECT_EQ(9, params.ColFromAngle(9_deg));
  EXPECT_EQ(9, params.ColFromAngle(10_deg));

  EXPECT_EQ(0, params.RowFromAngle(0_deg));
  EXPECT_EQ(1, params.RowFromAngle(1_deg));
  EXPECT_EQ(2, params.RowFromAngle(2_deg));
  EXPECT_EQ(3, params.RowFromAngle(3_deg));
  EXPECT_EQ(4, params.RowFromAngle(4_deg));
  EXPECT_EQ(5, params.RowFromAngle(5_deg));
  EXPECT_EQ(6, params.RowFromAngle(6_deg));
  EXPECT_EQ(7, params.RowFromAngle(7_deg));
  EXPECT_EQ(8, params.RowFromAngle(8_deg));
  EXPECT_EQ(9, params.RowFromAngle(9_deg));
  EXPECT_EQ(9, params.RowFromAngle(10_deg));
}

TEST(TestProjParams, from_file) {
  auto params_ptr = ProjectionParams::FromConfigFile(
      SOURCE_PATH + "/test/test_files/test_params.cfg");
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(64, params_ptr->rows());
  EXPECT_EQ(870, params_ptr->cols());
  float eps = 0.0001f;
  EXPECT_NEAR(360, params_ptr->h_span().ToDegrees(), eps);
  EXPECT_NEAR(26.9359, params_ptr->v_span().ToDegrees(), eps);
  EXPECT_NEAR(-1.9367, params_ptr->AngleFromRow(0).ToDegrees(), eps);
  EXPECT_NEAR(24.9992, params_ptr->AngleFromRow(63).ToDegrees(), eps);
}

TEST(TestProjParams, velodyne_16) {
  auto params_ptr = ProjectionParams::VLP_16();
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(16, params_ptr->rows());
  EXPECT_EQ(870, params_ptr->cols());
}

TEST(TestProjParams, velodyne_64) {
  auto params_ptr = ProjectionParams::HDL_64();
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(64, params_ptr->rows());
  EXPECT_EQ(870, params_ptr->cols());
}

TEST(TestProjParams, velodyne_32) {
  auto params_ptr = ProjectionParams::HDL_32();
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(32, params_ptr->rows());
  EXPECT_EQ(870, params_ptr->cols());
}

TEST(TestProjParams, velodyne_64_equal) {
  auto params_ptr = ProjectionParams::HDL_64_EQUAL();
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(64, params_ptr->rows());
  EXPECT_EQ(870, params_ptr->cols());
}


TEST(TestProjParams, full_sphere) {
  auto params_ptr = ProjectionParams::FullSphere(1_deg);
  EXPECT_EQ(true, params_ptr->valid());
  EXPECT_EQ(180, params_ptr->rows());
  EXPECT_EQ(360, params_ptr->cols());
}

