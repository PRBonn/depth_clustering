// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include "utils/pose.h"
#include <Eigen/Core>
#include <gtest/gtest.h>

using Eigen::Vector3f;

using namespace depth_clustering;

TEST(PoseTest, InitEmpty) {
  Pose pose;
  double eps = 0.000001;
  EXPECT_NEAR(0.0, pose.x(), eps);
  EXPECT_NEAR(0.0, pose.y(), eps);
  EXPECT_NEAR(0.0, pose.theta(), eps);
}

TEST(PoseTest, InitEmptyPtr) {
  Pose::Ptr pose(new Pose);
  double eps = 0.000001;
  EXPECT_NEAR(0.0, pose->x(), eps);
  EXPECT_NEAR(0.0, pose->y(), eps);
  EXPECT_NEAR(0.0, pose->theta(), eps);

  Eigen::Affine3f m;
  m.matrix() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  EXPECT_NEAR(m(0, 0), pose->matrix()(0, 0), eps);
  EXPECT_NEAR(m(1, 0), pose->matrix()(1, 0), eps);
  EXPECT_NEAR(m(0, 1), pose->matrix()(0, 1), eps);
  EXPECT_NEAR(m(1, 1), pose->matrix()(1, 1), eps);
  EXPECT_NEAR(m(2, 2), pose->matrix()(2, 2), eps);
}

TEST(PoseTest, InitCopy) {
  double eps = 0.000001;
  Vector3f vect(1.0f, 1.0f, 1.0f);
  Pose pose(vect);
  EXPECT_NEAR(1.0, pose.x(), eps);
  EXPECT_NEAR(1.0, pose.y(), eps);
  EXPECT_NEAR(1.0, pose.theta(), eps);
}

TEST(PoseTest, InitFull) {
  double eps = 0.000001;
  Pose pose(1, 1, 1);
  EXPECT_NEAR(1.0, pose.x(), eps);
  EXPECT_NEAR(1.0, pose.y(), eps);
  EXPECT_NEAR(1.0, pose.theta(), eps);
}

TEST(PoseTest, NormalizeTheta) {
  double eps = 0.000001;
  Pose pose(1, 1, 2 * M_PI);
  EXPECT_NEAR(1.0, pose.x(), eps);
  EXPECT_NEAR(1.0, pose.y(), eps);
  EXPECT_NEAR(0.0, pose.theta(), eps);

  Pose pose2(20, 20, -2 * M_PI);
  EXPECT_NEAR(20.0, pose2.x(), eps);
  EXPECT_NEAR(20.0, pose2.y(), eps);
  EXPECT_NEAR(0.0, pose2.theta(), eps);
}

TEST(PoseTest, Assign) {
  Pose pose;
  double eps = 0.000001;
  EXPECT_NEAR(0.0, pose.x(), eps);
  EXPECT_NEAR(0.0, pose.y(), eps);
  EXPECT_NEAR(0.0, pose.theta(), eps);
  Vector3f vect(1, 1, 2 * M_PI);
  pose = vect;
  EXPECT_NEAR(1.0, pose.x(), eps);
  EXPECT_NEAR(1.0, pose.y(), eps);
  EXPECT_NEAR(0.0, pose.theta(), eps);
}

TEST(PoseTest, FromTransform) {
  Eigen::Affine3f m;
  m.matrix() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  Pose pose(m);
  double eps = 0.000001;
  EXPECT_NEAR(0.0, pose.x(), eps);
  EXPECT_NEAR(0.0, pose.y(), eps);
  EXPECT_NEAR(0.0, pose.theta(), eps);

  Eigen::Affine3f m_rot;
  m_rot.matrix() << 0.5, sqrt(3) / 2, 0, 0, sqrt(3) / 2, 0.5, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 1;
  Pose pose_rot(m_rot);
  EXPECT_NEAR(0.0, pose_rot.x(), eps);
  EXPECT_NEAR(0.0, pose_rot.y(), eps);
  EXPECT_NEAR(M_PI / 3, pose_rot.theta(), eps);

  Eigen::Affine3f m_move;
  m_move.matrix() << 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
  Pose pose_move(m_move);
  EXPECT_NEAR(1.0, pose_move.x(), eps);
  EXPECT_NEAR(1.0, pose_move.y(), eps);
  EXPECT_NEAR(0, pose_move.theta(), eps);

  Eigen::Affine3f m_move_rot;
  m_move_rot.matrix() << 0.5, -sqrt(3) / 2, 0, 1, sqrt(3) / 2, 0.5, 0, 1, 0, 0,
      1, 0, 0, 0, 0, 1;
  Pose pose_move_rot(m_move_rot);
  EXPECT_NEAR(1.0, pose_move_rot.x(), eps);
  EXPECT_NEAR(1.0, pose_move_rot.y(), eps);
  EXPECT_NEAR(M_PI / 3, pose_move_rot.theta(), eps);

  m_move_rot.matrix() << 0.5, sqrt(3) / 2, 0, 1, -sqrt(3) / 2, 0.5, 0, 1, 0, 0,
      1, 0, 0, 0, 0, 1;
  pose_move_rot = m_move_rot;
  EXPECT_NEAR(1.0, pose_move_rot.x(), eps);
  EXPECT_NEAR(1.0, pose_move_rot.y(), eps);
  EXPECT_NEAR(-M_PI / 3, pose_move_rot.theta(), eps);
}

TEST(PoseTest, ToTransform) {
  Eigen::Affine3f m;
  m.matrix() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  Pose pose(m);
  double eps = 0.000001;
  EXPECT_NEAR(0.0, pose.x(), eps);
  EXPECT_NEAR(0.0, pose.y(), eps);
  EXPECT_NEAR(0.0, pose.theta(), eps);

  Eigen::Affine3f m_res = pose;
  EXPECT_NEAR(m(0, 0), m_res(0, 0), eps);
  EXPECT_NEAR(m(1, 1), m_res(1, 1), eps);
  EXPECT_NEAR(m(2, 2), m_res(2, 2), eps);

  Eigen::Affine3f m_rot;
  m_rot.matrix() << 0.5, -sqrt(3) / 2, 0, 0, sqrt(3) / 2, 0.5, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 1;
  Pose pose_rot(m_rot);
  EXPECT_NEAR(0.0, pose_rot.x(), eps);
  EXPECT_NEAR(0.0, pose_rot.y(), eps);
  EXPECT_NEAR(M_PI / 3, pose_rot.theta(), eps);

  m_res = pose_rot;
  EXPECT_NEAR(m_rot(0, 0), m_res(0, 0), eps);
  EXPECT_NEAR(m_rot(1, 0), m_res(1, 0), eps);
  EXPECT_NEAR(m_rot(0, 1), m_res(0, 1), eps);
  EXPECT_NEAR(m_rot(1, 1), m_res(1, 1), eps);
  EXPECT_NEAR(m_rot(2, 2), m_res(2, 2), eps);

  Eigen::Affine3f m_move;
  m_move.matrix() << 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
  Pose pose_move(m_move);
  EXPECT_NEAR(1.0, pose_move.x(), eps);
  EXPECT_NEAR(1.0, pose_move.y(), eps);
  EXPECT_NEAR(0, pose_move.theta(), eps);

  m_res = pose_move;
  EXPECT_NEAR(m_move(0, 0), m_res(0, 0), eps);
  EXPECT_NEAR(m_move(1, 1), m_res(1, 1), eps);
  EXPECT_NEAR(m_move(2, 2), m_res(2, 2), eps);

  Eigen::Affine3f m_move_rot;
  m_move_rot.matrix() << 0.5, -sqrt(3) / 2, 0, 1, sqrt(3) / 2, 0.5, 0, 1, 0, 0,
      1, 0, 0, 0, 0, 1;
  Pose pose_move_rot(m_move_rot);
  EXPECT_NEAR(1.0, pose_move_rot.x(), eps);
  EXPECT_NEAR(1.0, pose_move_rot.y(), eps);
  EXPECT_NEAR(M_PI / 3, pose_move_rot.theta(), eps);

  m_res = pose_move_rot;
  EXPECT_NEAR(m_move_rot(0, 0), m_res(0, 0), eps);
  EXPECT_NEAR(m_move_rot(1, 1), m_res(1, 1), eps);
  EXPECT_NEAR(m_move_rot(2, 2), m_res(2, 2), eps);
}

TEST(PoseTest, ToLocalFrameOf) {
  double eps = 0.000001;
  Pose pose1(1, 0, 0);
  Pose pose2(2, 1, M_PI / 2);
  pose2.ToLocalFrameOf(pose1);
  EXPECT_NEAR(1, pose2.x(), eps);
  EXPECT_NEAR(1, pose2.y(), eps);
  EXPECT_NEAR(M_PI / 2, pose2.theta(), eps);

  pose2 = Pose(2, 1, M_PI / 2);
  pose1.ToLocalFrameOf(pose2);
  EXPECT_NEAR(-1, pose1.x(), eps);
  EXPECT_NEAR(1, pose1.y(), eps);
  EXPECT_NEAR(-M_PI / 2, pose1.theta(), eps);
}

TEST(PoseTest, ToLocalFrameOfPtr) {
  double eps = 0.000001;
  Pose::Ptr pose1(new Pose(1, 0, 0));
  Pose::Ptr pose2(new Pose(2, 1, M_PI / 2));
  pose2->ToLocalFrameOf(*pose1);
  EXPECT_NEAR(1, pose2->x(), eps);
  EXPECT_NEAR(1, pose2->y(), eps);
  EXPECT_NEAR(M_PI / 2, pose2->theta(), eps);

  pose2.reset(new Pose(2, 1, M_PI / 2));
  pose1->ToLocalFrameOf(*pose2);
  EXPECT_NEAR(-1, pose1->x(), eps);
  EXPECT_NEAR(1, pose1->y(), eps);
  EXPECT_NEAR(-M_PI / 2, pose1->theta(), eps);
}

TEST(PoseTest, TestLikelihood) {
  double eps = 0.000001;
  Pose pose;
  EXPECT_NEAR(1.0, pose.likelihood(), eps);
  pose.SetLikelihood(0.5);
  EXPECT_NEAR(0.5, pose.likelihood(), eps);

  Pose pose2(pose);
  EXPECT_NEAR(0.5, pose2.likelihood(), eps);

  Pose pose3;
  EXPECT_NEAR(1.0, pose3.likelihood(), eps);
  pose3 = pose;
  EXPECT_NEAR(0.5, pose3.likelihood(), eps);
}

TEST(PoseTest, TestOperatorUnaryMinus) {
  double eps = 0.000001;
  Pose pose;
  pose.SetX(1);
  pose.SetY(1);
  pose.SetZ(1);
  Pose inverted = -pose;
  EXPECT_NEAR(pose.x(), -inverted.x(), eps);
  EXPECT_NEAR(pose.y(), -inverted.y(), eps);
  EXPECT_NEAR(pose.z(), -inverted.z(), eps);
}

TEST(PoseTest, T2V) {
  float eps = std::numeric_limits<float>::epsilon();
  Pose pose;
  pose.SetX(1);
  pose.SetY(1);
  pose.SetZ(1);
  auto vec = pose.ToVector6f();
  EXPECT_NEAR(1, vec[0], eps);
  EXPECT_NEAR(1, vec[1], eps);
  EXPECT_NEAR(1, vec[2], eps);
  EXPECT_NEAR(0, vec[3], eps);
  EXPECT_NEAR(0, vec[4], eps);
  EXPECT_NEAR(0, vec[5], eps);
}

TEST(PoseTest, V2T) {
  float eps = std::numeric_limits<float>::epsilon();
  Pose::Vector6f vec;
  vec << 1, 1, 1, 0, 0, 0;
  Pose pose = Pose::FromVector6f(vec);
  EXPECT_NEAR(1, pose.x(), eps);
  EXPECT_NEAR(1, pose.y(), eps);
  EXPECT_NEAR(1, pose.z(), eps);
}

TEST(PoseTest, V2T2V) {
  float eps = std::numeric_limits<float>::epsilon();
  Pose::Vector6f vec;
  vec << 1, 2, 3, M_PI / 3, M_PI / 4, M_PI / 2;
  Pose pose = Pose::FromVector6f(vec);
  auto vec_res = pose.ToVector6f();
  EXPECT_NEAR(vec[0], vec_res[0], eps);
  EXPECT_NEAR(vec[1], vec_res[1], eps);
  EXPECT_NEAR(vec[2], vec_res[2], eps);
  EXPECT_NEAR(vec[3], vec_res[3], eps);
  EXPECT_NEAR(vec[4], vec_res[4], eps);
  EXPECT_NEAR(vec[5], vec_res[5], eps);
}

// IMPORTANT!!! This will only die if NDEBUG is not defined!
// More: http://en.cppreference.com/w/cpp/error/assert
#ifndef NDEBUG
TEST(PoseDeathTest, TestLikelihood) {
  double eps = 0.000001;
  Pose pose;
  ASSERT_DEATH(pose.SetLikelihood(2), "");
  ASSERT_DEATH(pose.SetLikelihood(-1), "");
}
#endif
