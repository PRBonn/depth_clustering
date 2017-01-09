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

#ifndef SRC_UTILS_RICH_POINT_H_
#define SRC_UTILS_RICH_POINT_H_

#include <Eigen/Core>

namespace depth_clustering {

/**
 * @brief      A point class that holds additional ring information
 */
class RichPoint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RichPoint() {}
  explicit RichPoint(float x, float y, float z) : _point(x, y, z) {}
  explicit RichPoint(float x, float y, float z, uint16_t ring)
      : _point{x, y, z}, _ring{ring} {}
  explicit RichPoint(Eigen::Vector3f& eigen_vec) : _point(eigen_vec) {}
  ~RichPoint() {}

  inline int ring() const { return _ring; }
  inline float x() const { return _point.x(); }
  inline float y() const { return _point.y(); }
  inline float z() const { return _point.z(); }

  inline uint16_t& ring() { return _ring; }
  inline float& x() { return _point.x(); }
  inline float& y() { return _point.y(); }
  inline float& z() { return _point.z(); }

  inline const Eigen::Vector3f& AsEigenVector() const { return _point; }
  inline Eigen::Vector3f& AsEigenVector() { return _point; }

  inline float DistToSensor2D() const {
    return sqrt(_point.x() * _point.x() + _point.y() * _point.y());
  }

  inline float DistToSensor3D() const {
    return sqrt(_point.x() * _point.x() + _point.y() * _point.y() +
                _point.z() * _point.z());
  }

  RichPoint& operator=(const RichPoint& other);
  RichPoint& operator=(const Eigen::Vector3f& other);
  bool operator==(const RichPoint& other) const;

 private:
  Eigen::Vector3f _point = Eigen::Vector3f::Zero();
  uint16_t _ring = 0;
};

}  // namespace depth_clustering

#endif  // SRC_UTILS_RICH_POINT_H_
