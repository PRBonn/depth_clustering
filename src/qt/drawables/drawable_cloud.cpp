// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include "./drawable_cloud.h"
#include <GL/glut.h>

void DrawableCloud::Draw() const {
  if (!_cloud_ptr) {
    throw std::runtime_error("DrawableCloud has no cloud to draw.");
  }
  glPushMatrix();
  glBegin(GL_POINTS);
  glColor3f(_color[0], _color[1], _color[2]);
  for (const auto& point : _cloud_ptr->points()) {
    auto real_point = _cloud_ptr->pose() * point.AsEigenVector();
    glVertex3f(real_point.x(), real_point.y(), real_point.z());
  }
  glEnd();
  glPopMatrix();
}

DrawableCloud::Ptr DrawableCloud::FromCloud(const Cloud::ConstPtr& cloud,
                                            const Eigen::Vector3f& color) {
  return std::make_shared<DrawableCloud>(DrawableCloud(cloud, color));
}
