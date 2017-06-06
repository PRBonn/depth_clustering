// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_QT_DRAWABLES_OBJECT_PAINTER_H_
#define SRC_QT_DRAWABLES_OBJECT_PAINTER_H_

#include <communication/abstract_client.h>
#include <utils/cloud.h>
#include <utils/timer.h>

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>

#include "qt/drawables/drawable_cube.h"
#include "qt/viewer/viewer.h"

class ObjectPainter
    : public depth_clustering::AbstractClient<
          std::unordered_map<uint16_t, depth_clustering::Cloud>> {
  using Cloud = depth_clustering::Cloud;
  using Timer = depth_clustering::time_utils::Timer;

 public:
  void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds,
                           int client_id) override {
    if (!_viewer) { return; }
    Timer timer;
    for (const auto& kv : clouds) {
      const auto& cluster = kv.second;
      Eigen::Vector3f center = Eigen::Vector3f::Zero();
      Eigen::Vector3f extent = Eigen::Vector3f::Zero();
      Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                                std::numeric_limits<float>::lowest(),
                                std::numeric_limits<float>::lowest());
      Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max());
      for (const auto& point : cluster.points()) {
        center = center + point.AsEigenVector();
        min_point << std::min(min_point.x(), point.x()),
            std::min(min_point.y(), point.y()),
            std::min(min_point.z(), point.z());
        max_point << std::max(max_point.x(), point.x()),
            std::max(max_point.y(), point.y()),
            std::max(max_point.z(), point.z());
      }
      center /= cluster.size();
      if (min_point.x() < max_point.x()) { extent = max_point - min_point; }
      _viewer->AddDrawable(DrawableCube::Create(center, extent));
    }
    fprintf(stderr, "[TIMING]: Adding all boxes took %lu us\n",
            timer.measure(Timer::Units::Micro));
    _viewer->update();
    fprintf(stderr, "[TIMING]: Viewer updated in %lu us\n",
            timer.measure(Timer::Units::Micro));
  }

  explicit ObjectPainter(Viewer* viewer) : _viewer(viewer) {}
  virtual ~ObjectPainter() {}

 private:
  Viewer* _viewer = nullptr;
};

#endif  // SRC_QT_DRAWABLES_OBJECT_PAINTER_H_
