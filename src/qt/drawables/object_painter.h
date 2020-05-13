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

#ifndef SRC_QT_DRAWABLES_OBJECT_PAINTER_H_
#define SRC_QT_DRAWABLES_OBJECT_PAINTER_H_

#include "communication/abstract_client.h"
#include "qt/drawables/drawable_cube.h"
#include "qt/drawables/drawable_polygon3d.h"
#include "qt/viewer/viewer.h"

#include <utils/cloud.h>
#include <utils/timer.h>

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace depth_clustering {

class ObjectPainter
    : public depth_clustering::AbstractClient<
          std::unordered_map<uint16_t, depth_clustering::Cloud>> {
  using Cloud = depth_clustering::Cloud;
  using Timer = depth_clustering::time_utils::Timer;

 public:
  enum class OutlineType { kBox, kPolygon3d };

  explicit ObjectPainter(Viewer* viewer, OutlineType outline_type)
      : viewer_{viewer}, outline_type_{outline_type} {}

  void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds,
                           int id) override;

 private:
  static Drawable::UniquePtr CreateDrawableCube(
      const depth_clustering::Cloud& cloud);

  static Drawable::UniquePtr CreateDrawablePolygon3d(
      const depth_clustering::Cloud& cloud);

  Viewer* viewer_{nullptr};
  OutlineType outline_type_{OutlineType::kBox};
};

}  // namespace depth_clustering

#endif  // SRC_QT_DRAWABLES_OBJECT_PAINTER_H_
