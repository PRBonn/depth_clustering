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

#ifndef SRC_IMAGE_LABELERS_PIXEL_COORDS_H_
#define SRC_IMAGE_LABELERS_PIXEL_COORDS_H_

#include <cstdint>

namespace depth_clustering {

/**
 * @brief      Pixel coordinates structure
 */
struct PixelCoord {
  PixelCoord() : row(0), col(0) {}
  PixelCoord(int16_t row_, int16_t col_) : row(row_), col(col_) {}
  PixelCoord operator+(const PixelCoord& other) const {
    return PixelCoord(row + other.row, col + other.col);
  }

  int16_t row;
  int16_t col;
};

}  // namespace depth_clustering

#endif  // SRC_IMAGE_LABELERS_PIXEL_COORDS_H_
