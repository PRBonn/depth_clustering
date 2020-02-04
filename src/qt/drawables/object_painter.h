// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

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
