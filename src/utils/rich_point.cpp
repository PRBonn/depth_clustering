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
