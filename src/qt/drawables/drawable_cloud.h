// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#pragma once

#include "qt/drawables/drawable.h"
#include "utils/cloud.h"

class DrawableCloud : public Drawable {
 public:
  using Ptr = std::shared_ptr<DrawableCloud>;
  using Cloud = depth_clustering::Cloud;

  explicit DrawableCloud(const Cloud::ConstPtr& cloud,
                         const Eigen::Vector3f& color = Eigen::Vector3f::Ones())
      : _cloud_ptr{cloud}, _color{color} {}

  void Draw() const override;

  static DrawableCloud::Ptr FromCloud(
      const Cloud::ConstPtr& cloud,
      const Eigen::Vector3f& color = Eigen::Vector3f::Ones());

  ~DrawableCloud() override {}

 private:
  Cloud::ConstPtr _cloud_ptr = nullptr;
  Eigen::Vector3f _color = Eigen::Vector3f::Ones();
};

