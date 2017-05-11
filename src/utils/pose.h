// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SRC_UTILS_POSE_H_
#define SRC_UTILS_POSE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>

#include "utils/useful_typedefs.h"

namespace depth_clustering {

/**
 * @brief      Extends Eigen::Affine transform adding useful functionality to it
 */
class Pose : public Eigen::Affine3f {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Vector6f = Eigen::Matrix<float, 6, 1>;

  using Ptr = shared_ptr<Pose>;
  using ConstPtr = shared_ptr<const Pose>;
  using Base = Eigen::Affine3f;

  Pose() : Base(), _likelihood(1.0f) {
    this->matrix() = Eigen::Matrix4f::Identity();
  }

  Pose(const float x, const float y, const float theta) : Pose() {
    this->SetMatrixFromTheta(theta);
    this->SetX(x);
    this->SetY(y);
  }

  explicit Pose(const Eigen::Vector3f& v) : Pose(v(0), v(1), v(2)) {}

  explicit Pose(const Eigen::Affine3f& m) : Base(m), _likelihood(1.0) {}

  float x() const { return this->matrix()(0, 3); }
  float y() const { return this->matrix()(1, 3); }
  float z() const { return this->matrix()(2, 3); }

  float theta() const {
    const Eigen::Matrix4f& m = this->matrix();
    return m(1, 0) > 0 ? std::acos(m(0, 0)) : -std::acos(m(0, 0));
  }

  const float& likelihood() const { return _likelihood; }

  void SetX(float x) { this->matrix()(0, 3) = x; }
  void SetY(float y) { this->matrix()(1, 3) = y; }
  void SetZ(float z) { this->matrix()(2, 3) = z; }

  void SetTheta(float theta) { this->SetMatrixFromTheta(theta); }

  void SetPitch(float pitch) {
    Eigen::Affine3f& m = *this;
    m(0, 0) = std::cos(pitch);
    m(2, 2) = std::cos(pitch);
    m(0, 2) = -std::sin(pitch);
    m(2, 0) = std::sin(pitch);
  }
  void SetRoll(float roll) {
    Eigen::Affine3f& m = *this;
    m(1, 1) = std::cos(roll);
    m(2, 2) = std::cos(roll);
    m(1, 2) = -std::sin(roll);
    m(2, 1) = std::sin(roll);
  }
  void SetYaw(float yaw) { this->SetTheta(yaw); }

  void SetLikelihood(float likelihood) {
    assert(likelihood >= 0 && likelihood <= 1);
    _likelihood = likelihood;
  }

  void ToLocalFrameOf(const Pose& other) {
    this->matrix() = other.matrix().inverse() * this->matrix();
  }

  Pose InLocalFrameOf(const Pose& other) const {
    Pose pose(*this);
    pose.ToLocalFrameOf(other);
    return pose;
  }

  Pose::Ptr InLocalFrameOf(const Pose::Ptr& other) const {
    Pose::Ptr pose(new Pose(*this));
    pose->ToLocalFrameOf(*other);
    return pose;
  }

  Pose& operator=(const Eigen::Affine3f& other) {
    Base::operator=(other);
    this->_likelihood = 1.0;
    return *this;
  }

  Pose& operator=(const Pose& other) {
    this->matrix() = other.matrix();
    this->SetLikelihood(other.likelihood());
    return *this;
  }

  Pose& operator=(const Eigen::Matrix4f& other) {
    this->matrix() = other;
    return *this;
  }

  Pose& operator=(const Eigen::Vector3f& v) {
    this->matrix() = Eigen::Matrix4f::Identity();
    this->SetMatrixFromTheta(v(2));
    this->SetX(v.x());
    this->SetY(v.y());
    return *this;
  }

  Pose operator-() {
    Pose inverted;
    inverted.SetX(-this->x());
    inverted.SetY(-this->y());
    inverted.SetZ(-this->z());
    return inverted;
  }

  inline void Print2D() const {
    fprintf(stderr, "[%f, %f, %f]\n", this->x(), this->y(), this->theta());
  }

  inline void Print3D() const {
    fprintf(stderr, "[%f, %f, %f]\n", this->x(), this->y(), this->z());
  }

  static inline Pose FromVector6f(const Vector6f& v) {
    using Eigen::Vector3f;
    using Eigen::AngleAxisf;
    Pose T;
    T.setIdentity();
    T.translation() = v.head<3>();
    Eigen::Matrix3f m;
    m = AngleAxisf(v(3), Vector3f::UnitX()) *
        AngleAxisf(v(4), Vector3f::UnitY()) *
        AngleAxisf(v(5), Vector3f::UnitZ());
    T.linear() = m;
    return T;
  }

  inline Vector6f ToVector6f() const {
    Vector6f v;
    v.head<3>() = this->translation();
    v.block<3, 1>(3, 0) = this->linear().eulerAngles(0, 1, 2);
    return v;
  }

 private:
  void SetMatrixFromTheta(float theta) {
    Eigen::Affine3f& m = *this;
    m(0, 0) = std::cos(theta);
    m(1, 1) = std::cos(theta);
    m(0, 1) = -std::sin(theta);
    m(1, 0) = std::sin(theta);
  }

  float _likelihood;
};

}  // namespace depth_clustering

#endif  // SRC_UTILS_POSE_H_
