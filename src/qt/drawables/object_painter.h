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

  void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds,
                           int) override {
    if (!viewer_) {
      return;
    }
    Timer timer;
    for (const auto& kv : clouds) {
      const auto& cluster = kv.second;
      Drawable::UniquePtr drawable{nullptr};
      switch (outline_type_) {
        case OutlineType::kBox:
          drawable = CreateDrawableCube(cluster);
          break;
        case OutlineType::kPolygon3d:
          drawable = CreateDrawablePolygon3d(cluster);
          break;
      }
      if (drawable) {
        viewer_->AddDrawable(std::move(drawable));
      }
    }
    fprintf(stderr, "[TIMING]: Adding all boxes took %lu us\n",
            timer.measure(Timer::Units::Micro));
    viewer_->update();
    fprintf(stderr, "[TIMING]: Viewer updated in %lu us\n",
            timer.measure(Timer::Units::Micro));
  }

  explicit ObjectPainter(Viewer* viewer, OutlineType outline_type)
      : viewer_{viewer}, outline_type_{outline_type} {}

 private:
  Drawable::UniquePtr CreateDrawableCube(
      const depth_clustering::Cloud& cloud) const {
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f extent = Eigen::Vector3f::Zero();
    Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest());
    Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max());
    for (const auto& point : cloud.points()) {
      center = center + point.AsEigenVector();
      min_point << std::min(min_point.x(), point.x()),
          std::min(min_point.y(), point.y()),
          std::min(min_point.z(), point.z());
      max_point << std::max(max_point.x(), point.x()),
          std::max(max_point.y(), point.y()),
          std::max(max_point.z(), point.z());
    }
    center /= cloud.size();
    if (min_point.x() < max_point.x()) {
      extent = max_point - min_point;
    }
    return DrawableCube::Create(center, extent);
  }

  Drawable::UniquePtr CreateDrawablePolygon3d(
      const depth_clustering::Cloud& cloud) const {
    float min_z{std::numeric_limits<float>::max()};
    float max_z{std::numeric_limits<float>::lowest()};
    std::vector<cv::Point2f> cv_points;
    cv_points.reserve(cloud.size());
    std::vector<int> hull_indices;
    for (const auto& point : cloud.points()) {
      cv_points.emplace_back(cv::Point2f{point.x(), point.y()});
      min_z = std::min(min_z, point.z());
      max_z = std::max(max_z, point.z());
    }
    cv::convexHull(cv_points, hull_indices);
    DrawablePolygon3d::AlignedEigenVectors hull;
    hull.reserve(cloud.size());
    for (int index : hull_indices) {
      const auto& cv_point = cv_points[index];
      hull.emplace_back(cv_point.x, cv_point.y, min_z);
    }
    const float diff_z = max_z - min_z;
    if (diff_z < 0.3) {
      return nullptr;
    }
    return DrawablePolygon3d::Create(hull, diff_z);
  }

  Viewer* viewer_{nullptr};
  OutlineType outline_type_{OutlineType::kBox};
};

}  // namespace depth_clustering

#endif  // SRC_QT_DRAWABLES_OBJECT_PAINTER_H_
