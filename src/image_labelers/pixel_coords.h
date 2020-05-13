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
