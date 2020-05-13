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

#include "utils/rich_point.h"

namespace depth_clustering {

RichPoint& RichPoint::operator=(const RichPoint& other) {
  if (this != &other) {  // self-assignment check expected
    _point = other.AsEigenVector();
    _ring = other.ring();
  }
  return *this;
}

RichPoint& RichPoint::operator=(const Eigen::Vector3f& other) {
  this->_point = other;
  return *this;
}

bool RichPoint::operator==(const RichPoint& other) const {
  return this->x() == other.x() && this->y() == other.y() &&
         this->z() == other.z() && this->ring() == other.ring();
}

}  // namespace depth_clustering
