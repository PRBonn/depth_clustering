// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "image_labelers/diff_helpers/line_dist_diff.h"

using namespace depth_clustering;
using namespace cv;
using namespace std;

class TestLineDistDiff : public LineDistDiffPrecomputed {
 public:
  TestLineDistDiff(const cv::Mat* source_image, const ProjectionParams* params)
      : LineDistDiffPrecomputed{source_image, params} {}
  ~TestLineDistDiff() {}

  vector<float> GetRowAlphas() const { return _row_alphas; }
  vector<float> GetColAlphas() const { return _col_alphas; }
  cv::Mat GetDistsRow() const { return _dists_row; }
  cv::Mat GetDistsCol() const { return _dists_col; }
};

TEST(LineDistDiff, AlphasRows) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  TestLineDistDiff angle_diff_helper(&depth_image, &params);
  auto alphas_rows = angle_diff_helper.GetRowAlphas();
  EXPECT_EQ(4, alphas_rows.size());
  EXPECT_NEAR(Radians::FromDegrees(1).val(), alphas_rows[0], 0.001);
  EXPECT_NEAR(Radians::FromDegrees(1).val(), alphas_rows[1], 0.001);
  EXPECT_NEAR(Radians::FromDegrees(1).val(), alphas_rows[2], 0.001);
  EXPECT_NEAR(0, alphas_rows[3], 0.001);
}

TEST(LineDistDiff, AlphasCols) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  TestLineDistDiff angle_diff_helper(&depth_image, &params);
  auto alphas_cols = angle_diff_helper.GetColAlphas();
  EXPECT_EQ(4, alphas_cols.size());
  EXPECT_NEAR(Radians::FromDegrees(1).val(), alphas_cols[0], 0.001);
  EXPECT_NEAR(Radians::FromDegrees(1).val(), alphas_cols[1], 0.001);
  EXPECT_NEAR(Radians::FromDegrees(1).val(), alphas_cols[2], 0.001);
  EXPECT_NEAR(Radians::FromDegrees(1).val(), alphas_cols[3], 0.001);
}

TEST(LineDistDiff, LineDistCols) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  TestLineDistDiff angle_diff_helper(&depth_image, &params);
  auto beta_cols = angle_diff_helper.GetDistsCol();
  EXPECT_EQ(4, beta_cols.rows);
  EXPECT_EQ(4, beta_cols.cols);
  for (int r = 0; r < beta_cols.rows; ++r) {
    for (int c = 0; c < beta_cols.cols; ++c) {
      EXPECT_NEAR(1, beta_cols.at<float>(r, c), 0.01) << r << " " << c;
    }
  }
}

TEST(LineDistDiff, LineDistRows) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  TestLineDistDiff angle_diff_helper(&depth_image, &params);
  auto beta_rows = angle_diff_helper.GetDistsRow();
  EXPECT_EQ(4, beta_rows.rows);
  EXPECT_EQ(4, beta_rows.cols);
  for (int r = 0; r < beta_rows.rows - 1; ++r) {
    for (int c = 0; c < beta_rows.cols; ++c) {
      EXPECT_NEAR(1, beta_rows.at<float>(r, c), 0.01) << r << " " << c;
    }
  }
  EXPECT_NEAR(0.0f, beta_rows.at<float>(3, 0), 0.01);
  EXPECT_NEAR(0.0f, beta_rows.at<float>(3, 1), 0.01);
  EXPECT_NEAR(0.0f, beta_rows.at<float>(3, 2), 0.01);
  EXPECT_NEAR(0.0f, beta_rows.at<float>(3, 3), 0.01);
}

TEST(LineDistDiff, Start) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LineDistDiffPrecomputed angle_diff_helper(&depth_image, &params);
  auto eps = 0.01f;
  EXPECT_NEAR(1, angle_diff_helper.DiffAt(PixelCoord(0, 0), PixelCoord(1, 0)),
              eps);
  EXPECT_NEAR(1, angle_diff_helper.DiffAt(PixelCoord(1, 0), PixelCoord(0, 0)),
              eps);
  EXPECT_NEAR(1, angle_diff_helper.DiffAt(PixelCoord(0, 1), PixelCoord(0, 0)),
              eps);
  EXPECT_NEAR(1, angle_diff_helper.DiffAt(PixelCoord(0, 0), PixelCoord(0, 1)),
              eps);
}

TEST(LineDistDiff, Middle) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LineDistDiffPrecomputed angle_diff_helper(&depth_image, &params);
  auto eps = 0.01f;
  for (int r = 1; r < size - 1; ++r) {
    for (int c = 1; c < size - 1; ++c) {
      PixelCoord curr(r, c);
      PixelCoord next_r(r + 1, c);
      PixelCoord next_c(r, c + 1);
      EXPECT_NEAR(1, angle_diff_helper.DiffAt(curr, next_r), eps)
          << "row diff r=" << r << " r+1=" << r + 1 << " c=" << c;
      EXPECT_NEAR(1, angle_diff_helper.DiffAt(next_r, curr), eps)
          << "row diff r+1=" << r + 1 << " r=" << r << " c=" << c;
      EXPECT_NEAR(1, angle_diff_helper.DiffAt(curr, next_c), eps)
          << "col diff c=" << c << " c+1=" << c + 1 << " r=" << r;
      EXPECT_NEAR(1, angle_diff_helper.DiffAt(next_c, curr), eps)
          << "col diff c+1=" << c + 1 << " c=" << c << " r=" << r;
    }
  }
  EXPECT_NEAR(1, angle_diff_helper.DiffAt(PixelCoord(1, 1), PixelCoord(2, 1)),
              eps);
  EXPECT_NEAR(1, angle_diff_helper.DiffAt(PixelCoord(2, 1), PixelCoord(1, 1)),
              eps);
  EXPECT_NEAR(1, angle_diff_helper.DiffAt(PixelCoord(1, 2), PixelCoord(1, 1)),
              eps);
  EXPECT_NEAR(1, angle_diff_helper.DiffAt(PixelCoord(1, 1), PixelCoord(1, 2)),
              eps);
}

TEST(LineDistDiff, OverBorder) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LineDistDiffPrecomputed angle_diff_helper(&depth_image, &params);
  auto eps = 0.01f;
  EXPECT_NEAR(1, angle_diff_helper.DiffAt(PixelCoord(3, 3), PixelCoord(3, 0)),
              eps);
  EXPECT_NEAR(1, angle_diff_helper.DiffAt(PixelCoord(3, 0), PixelCoord(3, 3)),
              eps);
  EXPECT_NEAR(0, angle_diff_helper.DiffAt(PixelCoord(3, 3), PixelCoord(0, 3)),
              eps);
  EXPECT_NEAR(0, angle_diff_helper.DiffAt(PixelCoord(0, 3), PixelCoord(3, 3)),
              eps);
}

TEST(LineDistDiff, ColorVisualization) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LineDistDiffPrecomputed angle_diff_helper(&depth_image, &params);
  cv::Mat colors = angle_diff_helper.Visualize();
  auto eps = 0.1f;
  EXPECT_NEAR(255, colors.at<cv::Vec3b>(3, 3)[0], eps);
}
