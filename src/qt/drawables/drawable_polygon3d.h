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

#ifndef SRC_QT_DRAWABLES_DRAWABLE_POLYGON3D_H_
#define SRC_QT_DRAWABLES_DRAWABLE_POLYGON3D_H_

#include <Eigen/Core>
#include <vector>

#include "qt/drawables/drawable.h"
#include "utils/mem_utils.h"

namespace depth_clustering {

class DrawablePolygon3d : public Drawable {
 public:
  using AlignedEigenVectors =
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;
  DrawablePolygon3d(const AlignedEigenVectors& polygon, float height,
                    const Eigen::Vector3f& color)
      : polygon_{polygon}, height_{height}, color_{color} {}

  void Draw() const override;
  static DrawablePolygon3d::UniquePtr Create(
      const AlignedEigenVectors& polygon, float height,
      const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 0.5f, 0.2f)) {
    return mem_utils::make_unique<DrawablePolygon3d>(
        DrawablePolygon3d(polygon, height, color));
  }

  ~DrawablePolygon3d() override {}

 private:
  AlignedEigenVectors polygon_;
  float height_;
  Eigen::Vector3f color_;
};

}  // namespace depth_clustering

#endif  // SRC_QT_DRAWABLES_DRAWABLE_POLYGON3D_H_
