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

#ifndef SRC_QT_DRAWABLES_DRAWABLE_CUBE_H_
#define SRC_QT_DRAWABLES_DRAWABLE_CUBE_H_

#include <Eigen/Core>

#include "qt/drawables/drawable.h"
#include "utils/mem_utils.h"

namespace depth_clustering {

class DrawableCube : public Drawable {
 public:
  DrawableCube(const Eigen::Vector3f& center, const Eigen::Vector3f& scale,
               const Eigen::Vector3f& color)
      : _center{center}, _scale{scale}, _color{color} {}

  void Draw() const override;
  static DrawableCube::UniquePtr Create(
      const Eigen::Vector3f& center, const Eigen::Vector3f& scale,
      const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 0.5f, 0.2f)) {
    return mem_utils::make_unique<DrawableCube>(
        DrawableCube(center, scale, color));
  }

  ~DrawableCube() override {}

 private:
  Eigen::Vector3f _center;
  Eigen::Vector3f _scale;
  Eigen::Vector3f _color;
};

}  // namespace depth_clustering

#endif  // SRC_QT_DRAWABLES_DRAWABLE_CUBE_H_
