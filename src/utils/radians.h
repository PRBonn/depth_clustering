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

#ifndef SRC_UTILS_RADIANS_H_
#define SRC_UTILS_RADIANS_H_

#include <math.h>
#include <stdio.h>
#include <limits>

namespace depth_clustering {
class Radians;
}

// this class provides user defined literals.
// https://akrzemi1.wordpress.com/2012/08/12/user-defined-literals-part-i/
// forward declare Radians class and literals
constexpr depth_clustering::Radians operator"" _rad(long double angle);
constexpr depth_clustering::Radians operator"" _deg(
    unsigned long long int angle);
constexpr depth_clustering::Radians operator"" _deg(long double angle);

namespace depth_clustering {

class Radians {
 public:
  class IsRadians {};  // a tag to prevent using raw constructor
  Radians() : _raw_angle{0}, _valid{false} {}
  explicit constexpr Radians(IsRadians, float angle)
      : _raw_angle{angle}, _valid{true} {}

  inline float val() const { return _raw_angle; }
  inline bool valid() const { return _valid; }

  static Radians FromRadians(float radians) {
    return Radians{IsRadians{}, radians};
  }

  static Radians FromDegrees(float angle) {
    return Radians{IsRadians{}, static_cast<float>(angle * M_PI / 180.)};
  }

  float ToDegrees() const { return 180. * _raw_angle / M_PI; }

  Radians operator-(const Radians& other) const {
    return FromRadians(_raw_angle - other._raw_angle);
  }

  Radians operator+(const Radians& other) const {
    return FromRadians(_raw_angle + other._raw_angle);
  }

  Radians operator+=(const Radians& other) {
    this->_raw_angle += other._raw_angle;
    return *this;
  }

  Radians operator-=(const Radians& other) {
    this->_raw_angle -= other._raw_angle;
    return *this;
  }

  Radians operator/(const float& num) const {
    return FromRadians(_raw_angle / num);
  }

  float operator/(const Radians& other) const {
    return _raw_angle / other._raw_angle;
  }

  Radians operator*(const float& num) const {
    return FromRadians(_raw_angle * num);
  }

  Radians operator-() { return FromRadians(-_raw_angle); }

  bool operator<(const Radians& other) const {
    return _raw_angle <
           other._raw_angle - std::numeric_limits<float>::epsilon();
  }
  bool operator>(const Radians& other) const {
    return _raw_angle >
           other._raw_angle + std::numeric_limits<float>::epsilon();
  }

  void Normalize(const Radians& from = 0_deg, const Radians& to = 360_deg) {
    float diff = (to - from).val();
    while (_raw_angle < from.val()) {
      _raw_angle += diff;
    }
    while (_raw_angle > to.val()) {
      _raw_angle -= diff;
    }
  }

  Radians Normalize(const Radians& from = 0_deg,
                    const Radians& to = 360_deg) const {
    Radians new_angle = FromRadians(_raw_angle);
    new_angle.Normalize(from, to);
    return new_angle;
  }

  static Radians Abs(const Radians& angle) {
    return Radians::FromRadians(fabs(angle._raw_angle));
  }

  static Radians Floor(const Radians& angle) {
    return Radians::FromDegrees(floor(angle.ToDegrees()));
  }

 protected:
  float _raw_angle;
  bool _valid;
};

}  // namespace depth_clustering

constexpr depth_clustering::Radians operator"" _rad(long double angle) {
  return depth_clustering::Radians{depth_clustering::Radians::IsRadians{},
                                   static_cast<float>(angle)};
}

constexpr depth_clustering::Radians operator"" _deg(
    unsigned long long int angle) {
  return depth_clustering::Radians{depth_clustering::Radians::IsRadians{},
                                   static_cast<float>(angle * M_PI / 180.0)};
}

constexpr depth_clustering::Radians operator"" _deg(long double angle) {
  return depth_clustering::Radians{depth_clustering::Radians::IsRadians{},
                                   static_cast<float>(angle * M_PI / 180.0)};
}

#endif  // SRC_UTILS_RADIANS_H_
