// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include <gtest/gtest.h>

#include "cmake_config.h"
#include "projections/projection_params.h"

using namespace depth_clustering;

TEST(TestProjParams, test_init) {
  ProjectionParams params;
  EXPECT_EQ(false, params.valid());
}

TEST(TestProjParams, from_angle) {
  ProjectionParams params;
  params.SetSpan(0_deg, 10_deg, 1_deg, ProjectionParams::Direction::VERTICAL);
  params.SetSpan(0_deg, 10_deg, 1_deg, ProjectionParams::Direction::HORIZONTAL);
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
