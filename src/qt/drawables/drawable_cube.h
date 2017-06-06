// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_QT_DRAWABLES_DRAWABLE_CUBE_H_
#define SRC_QT_DRAWABLES_DRAWABLE_CUBE_H_

#include <Eigen/Core>

#include "qt/drawables/drawable.h"

class DrawableCube : public Drawable {
 public:
  DrawableCube(const Eigen::Vector3f& center, const Eigen::Vector3f& scale,
               const Eigen::Vector3f& color)
      : _center{center}, _scale{scale}, _color{color} {}

  void Draw() const override;
  static DrawableCube::Ptr Create(
      const Eigen::Vector3f& center, const Eigen::Vector3f& scale,
      const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 0.5f, 0.2f)) {
    return std::make_shared<DrawableCube>(DrawableCube(center, scale, color));
  }

  ~DrawableCube() override {}

 private:
  Eigen::Vector3f _center;
  Eigen::Vector3f _scale;
  Eigen::Vector3f _color;
};

#endif  // SRC_QT_DRAWABLES_DRAWABLE_CUBE_H_
