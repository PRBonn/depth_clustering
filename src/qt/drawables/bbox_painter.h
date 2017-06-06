// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_QT_DRAWABLES_BBOX_PAINTER_H_
#define SRC_QT_DRAWABLES_BBOX_PAINTER_H_

#include <communication/abstract_client.h>
#include <utils/cloud.h>
#include <utils/timer.h>

#include <algorithm>
#include <limits>
#include <vector>

#include "qt/drawables/drawable_cube.h"
#include "qt/viewer/viewer.h"
#include "utils/bbox.h"

namespace dc = depth_clustering;

class BBoxPainter : public dc::AbstractClient<std::vector<Bbox>> {
 public:
  void OnNewObjectReceived(const std::vector<Bbox>& bboxes,
                           int client_id) override {
    dc::time_utils::Timer timer;
    for (const auto& bbox : bboxes) {
      if (_viewer) {
        _viewer->AddDrawable(DrawableCube::Create(bbox.center(), bbox.scale()));
      }
    }
    fprintf(stderr, "[TIMING]: Adding all boxes took %lu us\n",
            timer.measure(dc::time_utils::Timer::Units::Micro));
    _viewer->update();
    fprintf(stderr, "[TIMING]: Viewer updated in %lu us\n",
            timer.measure(dc::time_utils::Timer::Units::Micro));
  }

  explicit BBoxPainter(Viewer* viewer) : _viewer(viewer) {}
  virtual ~BBoxPainter() {}

 private:
  Viewer* _viewer = nullptr;
};

#endif  // SRC_QT_DRAWABLES_BBOX_PAINTER_H_
