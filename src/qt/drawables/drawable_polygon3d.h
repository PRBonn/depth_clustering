// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

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
