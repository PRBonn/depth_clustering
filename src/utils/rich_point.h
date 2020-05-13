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

#ifndef SRC_UTILS_RICH_POINT_H_
#define SRC_UTILS_RICH_POINT_H_

#include <Eigen/Core>
#include <vector>

namespace depth_clustering {

/**
 * @brief      A point class that holds additional ring information
 */
class RichPoint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using AlignedVector =
      std::vector<RichPoint, Eigen::aligned_allocator<RichPoint>>;

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
