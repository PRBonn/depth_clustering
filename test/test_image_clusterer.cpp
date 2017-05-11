// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include <gtest/gtest.h>
#include <opencv/cv.h>

#include <string>
#include <vector>

#include "image_labelers/dijkstra_image_labeler.h"
#include "image_labelers/linear_image_labeler.h"

using std::string;
using std::vector;
using cv::Mat;
using cv::DataType;

using namespace depth_clustering;

TEST(LinearImageLabeler, NeighborhoodTest) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  Radians threshold = 20_deg;
  ProjectionParams params;
  LinearImageLabeler<1, 1> labeler(depth_image, params, threshold);
  EXPECT_EQ(4, labeler.Neighborhood.size());

  EXPECT_EQ(-1, labeler.Neighborhood[0].row);
  EXPECT_EQ(0, labeler.Neighborhood[0].col);

  EXPECT_EQ(1, labeler.Neighborhood[1].row);
  EXPECT_EQ(0, labeler.Neighborhood[1].col);

  EXPECT_EQ(0, labeler.Neighborhood[2].row);
  EXPECT_EQ(-1, labeler.Neighborhood[2].col);

  EXPECT_EQ(0, labeler.Neighborhood[3].row);
  EXPECT_EQ(1, labeler.Neighborhood[3].col);
}

TEST(LinearImageLabeler, NeighborhoodTestBigger) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  Radians threshold = 20_deg;
  ProjectionParams params;
  LinearImageLabeler<2, 1> labeler(depth_image, params, threshold);
  EXPECT_EQ(6, labeler.Neighborhood.size());

  EXPECT_EQ(-2, labeler.Neighborhood[0].row);
  EXPECT_EQ(0, labeler.Neighborhood[0].col);

  EXPECT_EQ(2, labeler.Neighborhood[1].row);
  EXPECT_EQ(0, labeler.Neighborhood[1].col);

  EXPECT_EQ(-1, labeler.Neighborhood[2].row);
  EXPECT_EQ(0, labeler.Neighborhood[2].col);

  EXPECT_EQ(1, labeler.Neighborhood[3].row);
  EXPECT_EQ(0, labeler.Neighborhood[3].col);

  EXPECT_EQ(0, labeler.Neighborhood[4].row);
  EXPECT_EQ(-1, labeler.Neighborhood[4].col);

  EXPECT_EQ(0, labeler.Neighborhood[5].row);
  EXPECT_EQ(1, labeler.Neighborhood[5].col);
}

TEST(LinearImageLabeler, UniformDepth) {
  int size = 4;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  Radians threshold = 20_deg;
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LinearImageLabeler<> labeler(depth_image, params, threshold);
  labeler.ComputeLabels(DiffFactory::DiffType::ANGLES);
  auto label_image = labeler.GetLabelImage();
  for (int r = 0; r < size; ++r) {
    for (int c = 0; c < size; ++c) {
      EXPECT_EQ(1, label_image->at<uint16_t>(r, c)) << "happens when r = " << r
                                                    << ", c = " << c;
    }
  }
}

// the image looks approx like this:
//      depth               labels
// 0.0 0.1 0.2 0.3 0.4     1 1 1 1 1
// 0.1 0.2 0.3 0.4 0.5     1 1 1 1 1
// 0.2 0.3 0.4 0.5 0.6     1 1 1 1 1
// 0.3 0.4 0.5 0.6 0.7     1 1 1 1 1
// 0.4 0.5 0.6 0.7 0.8     1 1 1 1 1
//
TEST(LinearImageLabeler, GradientDepth) {
  int size = 5;
  Mat depth_image = cv::Mat::zeros(size, size, DataType<float>::type);
  for (int row = 0; row < size; ++row) {
    for (int col = 0; col < size; ++col) {
      depth_image.at<float>(row, col) = 10.0f + (row + col) * 0.1;
    }
  }
  Radians threshold = 20_deg;
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LinearImageLabeler<> labeler(depth_image, params, threshold);
  labeler.ComputeLabels(DiffFactory::DiffType::ANGLES);
  auto label_image = labeler.GetLabelImage();
  for (int r = 0; r < size; ++r) {
    for (int c = 0; c < size; ++c) {
      ASSERT_EQ(1, label_image->at<uint16_t>(r, c));
    }
  }
}

// the image looks approx like this:
//            depth             expected labels
// 0.0 | 0.0 | 0.5 | 0.0 | 0.0     1 1 2 1 1
// 0.0 | 0.0 | 0.5 | 0.0 | 0.0     1 1 2 1 1
// 0.0 | 0.0 | 0.5 | 0.0 | 0.0     1 1 2 1 1
// 0.0 | 0.0 | 0.5 | 0.0 | 0.0     1 1 2 1 1
// 0.0 | 0.0 | 0.5 | 0.0 | 0.0     1 1 2 1 1
//
TEST(LinearImageLabeler, TwoLabelImageVertical) {
  int size = 5;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  for (int i = 0; i < size; ++i) {
    depth_image.at<float>(i, 2) = 0.5f;
  }
  Radians threshold = 20_deg;
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LinearImageLabeler<1, 1> labeler(depth_image, params, threshold);
  labeler.ComputeLabels(DiffFactory::DiffType::ANGLES);
  auto label_image = labeler.GetLabelImage();
  EXPECT_EQ(2, label_image->at<uint16_t>(0, 2));
  EXPECT_EQ(2, label_image->at<uint16_t>(4, 2));

  EXPECT_EQ(1, label_image->at<uint16_t>(0, 0));
  EXPECT_EQ(1, label_image->at<uint16_t>(4, 0));

  EXPECT_EQ(1, label_image->at<uint16_t>(0, 3));
  EXPECT_EQ(1, label_image->at<uint16_t>(4, 3));

  EXPECT_EQ(1, label_image->at<uint16_t>(0, 4));
  EXPECT_EQ(1, label_image->at<uint16_t>(4, 4));
}

// the image looks approx like this:
//            depth             expected labels
// 0.0 | 0.0 | 0.0 | 0.0 | 0.0     1 1 1 1 1
// 0.0 | 0.0 | 0.0 | 0.0 | 0.0     1 1 1 1 1
// 0.5 | 0.5 | 0.5 | 0.5 | 0.5     2 2 2 2 2
// 0.0 | 0.0 | 0.0 | 0.0 | 0.0     3 3 3 3 3
// 0.0 | 0.0 | 0.0 | 0.0 | 0.0     3 3 3 3 3
//
TEST(LinearImageLabeler, TwoLabelImageHorizontal) {
  int size = 5;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  for (int i = 0; i < size; ++i) {
    depth_image.at<float>(2, i) = 0.5f;
  }
  Radians threshold = 20_deg;
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LinearImageLabeler<1, 1> labeler(depth_image, params, threshold);
  labeler.ComputeLabels(DiffFactory::DiffType::ANGLES);
  auto label_image = labeler.GetLabelImage();
  ASSERT_EQ(2, label_image->at<uint16_t>(2, 0));
  ASSERT_EQ(2, label_image->at<uint16_t>(2, 4));

  ASSERT_EQ(1, label_image->at<uint16_t>(0, 0));
  ASSERT_EQ(1, label_image->at<uint16_t>(0, 3));

  ASSERT_EQ(3, label_image->at<uint16_t>(4, 0));
  ASSERT_EQ(3, label_image->at<uint16_t>(4, 3));
}

// the image looks approx like this:
//            depth             expected labels
// 0.5 | 0.0 | 0.5 | 0.0 | 0.5     1 2 3 2 1
// 0.0 | 0.0 | 0.0 | 0.0 | 0.5     2 2 2 2 1
// 0.5 | 0.5 | 0.0 | 0.5 | 0.5     1 1 2 1 1
// 0.0 | 0.0 | 0.0 | 0.5 | 0.0     2 2 2 1 2
// 0.0 | 0.0 | 0.0 | 0.5 | 0.0     2 2 2 1 2
//
TEST(LinearImageLabeler, MultiClassHard) {
  int size = 5;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  depth_image.at<float>(0, 0) = 0.5f;
  depth_image.at<float>(2, 0) = 0.5f;
  depth_image.at<float>(2, 1) = 0.5f;
  depth_image.at<float>(0, 2) = 0.5f;
  depth_image.at<float>(2, 3) = 0.5f;
  depth_image.at<float>(3, 3) = 0.5f;
  depth_image.at<float>(4, 3) = 0.5f;
  depth_image.at<float>(1, 4) = 0.5f;
  depth_image.at<float>(0, 4) = 0.5f;
  depth_image.at<float>(2, 4) = 0.5f;

  Radians threshold = 20_deg;
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LinearImageLabeler<1, 1> labeler(depth_image, params, threshold);
  labeler.ComputeLabels(DiffFactory::DiffType::ANGLES);
  auto label_image = labeler.GetLabelImage();

  ASSERT_EQ(1, label_image->at<uint16_t>(0, 0));
  ASSERT_EQ(1, label_image->at<uint16_t>(2, 0));
  ASSERT_EQ(1, label_image->at<uint16_t>(2, 1));
  ASSERT_EQ(1, label_image->at<uint16_t>(2, 3));
  ASSERT_EQ(1, label_image->at<uint16_t>(3, 3));
  ASSERT_EQ(1, label_image->at<uint16_t>(4, 3));
  ASSERT_EQ(1, label_image->at<uint16_t>(1, 4));
  ASSERT_EQ(1, label_image->at<uint16_t>(2, 4));
  ASSERT_EQ(1, label_image->at<uint16_t>(0, 4));

  ASSERT_EQ(2, label_image->at<uint16_t>(1, 0));
  ASSERT_EQ(2, label_image->at<uint16_t>(1, 1));
  ASSERT_EQ(2, label_image->at<uint16_t>(4, 4));

  ASSERT_EQ(3, label_image->at<uint16_t>(0, 2));
}

// the image looks approx like this:
//            depth             expected labels
// 0.0 | 0.0 | 0.5 | 0.0 | 0.0     1 1 2 1 1
// 0.0 | 0.0 | 0.5 | 0.0 | 0.0     1 1 2 1 1
// 0.0 | 0.0 | 0.5 | 0.0 | 0.0     1 1 2 1 1
// 0.0 | 0.0 | 0.5 | 0.0 | 0.0     1 1 2 1 1
// 0.0 | 0.0 | 0.5 | 0.0 | 0.0     1 1 2 1 1
//
TEST(DijkstraImageLabeler, TwoLabelImageVertical) {
  int size = 5;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  for (int i = 0; i < size; ++i) {
    depth_image.at<float>(i, 2) = 0.5f;
  }
  Radians threshold = 20_deg;
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LinearImageLabeler<1, 1> labeler(depth_image, params, threshold);
  labeler.ComputeLabels(DiffFactory::DiffType::ANGLES);
  auto label_image = labeler.GetLabelImage();
  EXPECT_EQ(2, label_image->at<uint16_t>(0, 2));
  EXPECT_EQ(2, label_image->at<uint16_t>(4, 2));

  EXPECT_EQ(1, label_image->at<uint16_t>(0, 0));
  EXPECT_EQ(1, label_image->at<uint16_t>(4, 0));

  EXPECT_EQ(1, label_image->at<uint16_t>(0, 3));
  EXPECT_EQ(1, label_image->at<uint16_t>(4, 3));

  EXPECT_EQ(1, label_image->at<uint16_t>(0, 4));
  EXPECT_EQ(1, label_image->at<uint16_t>(4, 4));
}

// the image looks approx like this:
//            depth             expected labels
// 0.0 | 0.0 | 0.0 | 0.0 | 0.0     1 1 1 1 1
// 0.0 | 0.0 | 0.0 | 0.0 | 0.0     1 1 1 1 1
// 0.5 | 0.5 | 0.5 | 0.5 | 0.5     2 2 2 2 2
// 0.0 | 0.0 | 0.0 | 0.0 | 0.0     3 3 3 3 3
// 0.0 | 0.0 | 0.0 | 0.0 | 0.0     3 3 3 3 3
//
TEST(DijkstraImageLabeler, TwoLabelImageHorizontal) {
  int size = 5;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  for (int i = 0; i < size; ++i) {
    depth_image.at<float>(2, i) = 0.5f;
  }
  Radians threshold = 20_deg;
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LinearImageLabeler<1, 1> labeler(depth_image, params, threshold);
  labeler.ComputeLabels(DiffFactory::DiffType::ANGLES);
  auto label_image = labeler.GetLabelImage();
  ASSERT_EQ(2, label_image->at<uint16_t>(2, 0));
  ASSERT_EQ(2, label_image->at<uint16_t>(2, 4));

  ASSERT_EQ(1, label_image->at<uint16_t>(0, 0));
  ASSERT_EQ(1, label_image->at<uint16_t>(0, 3));

  ASSERT_EQ(3, label_image->at<uint16_t>(4, 0));
  ASSERT_EQ(3, label_image->at<uint16_t>(4, 3));
}

// the image looks approx like this:
//            depth             expected labels
// 0.5 | 0.0 | 0.5 | 0.0 | 0.5     1 2 3 2 1
// 0.0 | 0.0 | 0.0 | 0.0 | 0.5     2 2 2 2 1
// 0.5 | 0.5 | 0.0 | 0.5 | 0.5     1 1 2 1 1
// 0.0 | 0.0 | 0.0 | 0.5 | 0.0     2 2 2 1 2
// 0.0 | 0.0 | 0.0 | 0.5 | 0.0     2 2 2 1 2
//
TEST(DijkstraImageLabeler, MultiClassHard) {
  int size = 5;
  Mat depth_image = cv::Mat::ones(size, size, DataType<float>::type);
  depth_image.at<float>(0, 0) = 0.5f;
  depth_image.at<float>(2, 0) = 0.5f;
  depth_image.at<float>(2, 1) = 0.5f;
  depth_image.at<float>(0, 2) = 0.5f;
  depth_image.at<float>(2, 3) = 0.5f;
  depth_image.at<float>(3, 3) = 0.5f;
  depth_image.at<float>(4, 3) = 0.5f;
  depth_image.at<float>(1, 4) = 0.5f;
  depth_image.at<float>(0, 4) = 0.5f;
  depth_image.at<float>(2, 4) = 0.5f;

  Radians threshold = 20_deg;
  ProjectionParams params;
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::VERTICAL);
  params.SetSpan(SpanParams(0_deg, 1_deg * size, size),
                 SpanParams::Direction::HORIZONTAL);
  LinearImageLabeler<1, 1> labeler(depth_image, params, threshold);
  labeler.ComputeLabels(DiffFactory::DiffType::ANGLES);
  auto label_image = labeler.GetLabelImage();

  ASSERT_EQ(1, label_image->at<uint16_t>(0, 0));
  ASSERT_EQ(1, label_image->at<uint16_t>(2, 0));
  ASSERT_EQ(1, label_image->at<uint16_t>(2, 1));
  ASSERT_EQ(1, label_image->at<uint16_t>(2, 3));
  ASSERT_EQ(1, label_image->at<uint16_t>(3, 3));
  ASSERT_EQ(1, label_image->at<uint16_t>(4, 3));
  ASSERT_EQ(1, label_image->at<uint16_t>(1, 4));
  ASSERT_EQ(1, label_image->at<uint16_t>(2, 4));
  ASSERT_EQ(1, label_image->at<uint16_t>(0, 4));

  ASSERT_EQ(2, label_image->at<uint16_t>(1, 0));
  ASSERT_EQ(2, label_image->at<uint16_t>(1, 1));
  ASSERT_EQ(2, label_image->at<uint16_t>(4, 4));

  ASSERT_EQ(3, label_image->at<uint16_t>(0, 2));
}
