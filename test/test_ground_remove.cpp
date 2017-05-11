// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include "ground_removal/depth_ground_remover.h"

using namespace depth_clustering;

class TestDepthGroundRemover : public DepthGroundRemover {
 public:
  explicit TestDepthGroundRemover(const ProjectionParams& proj_params,
                                  int window_size = 5)
      : DepthGroundRemover(proj_params, 5_deg, window_size) {}

  cv::Mat CreateAngleImage(const cv::Mat& depth_image) {
    return DepthGroundRemover::CreateAngleImage(depth_image);
  }

  cv::Mat GetSavitskyGolayKernel(int window_size) {
    return DepthGroundRemover::GetSavitskyGolayKernel(window_size);
  }

  Radians GetLineAngle(const cv::Mat& depth_image, int col, int row_curr,
                       int row_neigh) {
    return DepthGroundRemover::GetLineAngle(depth_image, col, row_curr,
                                            row_neigh);
  }
};

TEST(TestDepthGroundRemover, get_sav_gol_kernel) {
  ProjectionParams proj_params;
  TestDepthGroundRemover remover{proj_params};
  auto kernel = remover.GetSavitskyGolayKernel(5);
  float eps = std::numeric_limits<float>::epsilon();
  EXPECT_NEAR(-3.0f / 35.0f, kernel.at<float>(0, 0), eps);
  EXPECT_NEAR(12.0f / 35.0f, kernel.at<float>(0, 1), eps);
  EXPECT_NEAR(17.0f / 35.0f, kernel.at<float>(0, 2), eps);
  EXPECT_NEAR(12.0f / 35.0f, kernel.at<float>(0, 3), eps);
  EXPECT_NEAR(-3.0f / 35.0f, kernel.at<float>(0, 4), eps);

  kernel = remover.GetSavitskyGolayKernel(7);
  EXPECT_NEAR(-2.0f / 21.0f, kernel.at<float>(0, 0), eps);
  EXPECT_NEAR(3.0f / 21.0f, kernel.at<float>(0, 1), eps);
  EXPECT_NEAR(6.0f / 21.0f, kernel.at<float>(0, 2), eps);
  EXPECT_NEAR(7.0f / 21.0f, kernel.at<float>(0, 3), eps);
  EXPECT_NEAR(6.0f / 21.0f, kernel.at<float>(0, 4), eps);
  EXPECT_NEAR(3.0f / 21.0f, kernel.at<float>(0, 5), eps);
  EXPECT_NEAR(-2.0f / 21.0f, kernel.at<float>(0, 6), eps);

  kernel = remover.GetSavitskyGolayKernel(9);
  EXPECT_NEAR(-21.0f / 231.0f, kernel.at<float>(0, 0), eps);
  EXPECT_NEAR(59.0f / 231.0f, kernel.at<float>(0, 4), eps);

  kernel = remover.GetSavitskyGolayKernel(11);
  EXPECT_NEAR(-36.0f / 429.0f, kernel.at<float>(0, 0), eps);
  EXPECT_NEAR(89.0f / 429.0f, kernel.at<float>(0, 5), eps);
}

TEST(TestDepthGroundRemoverDeath, get_sav_gol_kernel_even) {
  ProjectionParams proj_params;
  TestDepthGroundRemover remover{proj_params};
  try {
    auto kernel = remover.GetSavitskyGolayKernel(2);
    FAIL();
  } catch (const std::logic_error& error) {
    std::string msg = "only odd window size allowed";
    EXPECT_EQ(msg, error.what());
  }
}

TEST(TestDepthGroundRemoverDeath, get_sav_gol_kernel_bad_window_size) {
  ProjectionParams proj_params;
  TestDepthGroundRemover remover{proj_params};
  try {
    auto kernel = remover.GetSavitskyGolayKernel(131);
    FAIL();
  } catch (const std::logic_error& error) {
    std::string msg = "bad window size";
    EXPECT_EQ(msg, error.what());
  }
}

TEST(TestDepthGroundRemover, trivial_depth_image) {
  ProjectionParams proj_params;
  proj_params.SetSpan(SpanParams(0_deg, 5_deg, 1_deg),
                      SpanParams::Direction::VERTICAL);
  proj_params.SetSpan(SpanParams(0_deg, 5_deg, 1_deg),
                      SpanParams::Direction::HORIZONTAL);
  TestDepthGroundRemover remover{proj_params};
  auto depth_image = cv::Mat::zeros(5, 5, CV_32F);
  auto res = remover.CreateAngleImage(depth_image);
  auto eps = std::numeric_limits<float>::epsilon();
  EXPECT_NEAR(0.0f, res.at<float>(0, 0), eps);
  EXPECT_NEAR(0.0f, res.at<float>(1, 1), eps);
  EXPECT_NEAR(0.0f, res.at<float>(2, 2), eps);
  EXPECT_NEAR(0.0f, res.at<float>(3, 3), eps);
  EXPECT_NEAR(0.0f, res.at<float>(4, 4), eps);
}

TEST(TestDepthGroundRemover, trivial_depth_image_vertical) {
  ProjectionParams proj_params;
  Radians start_angle = 0_deg;
  Radians end_angle = 5_deg;
  Radians step = 1_deg;
  proj_params.SetSpan(SpanParams(start_angle, end_angle, step),
                      SpanParams::Direction::VERTICAL);
  proj_params.SetSpan(SpanParams(start_angle, end_angle, step),
                      SpanParams::Direction::HORIZONTAL);
  TestDepthGroundRemover remover{proj_params};
  cv::Mat depth_image =
      cv::Mat::zeros(proj_params.rows(), proj_params.cols(), CV_32F);
  for (int r = 0; r < depth_image.rows; ++r) {
    for (int c = 0; c < depth_image.cols; ++c) {
      depth_image.at<float>(r, c) = 1 / cos((start_angle + step * r).val());
    }
  }
  auto res = remover.CreateAngleImage(depth_image);
  auto expected = 90_deg;
  auto eps = 0.0001f;
  EXPECT_NEAR(0.0f, res.at<float>(0, 0), eps);
  EXPECT_NEAR(expected.val(), res.at<float>(1, 0), eps);
  EXPECT_NEAR(expected.val(), res.at<float>(2, 0), eps);
  EXPECT_NEAR(expected.val(), res.at<float>(3, 0), eps);
  EXPECT_NEAR(expected.val(), res.at<float>(4, 0), eps);
}
