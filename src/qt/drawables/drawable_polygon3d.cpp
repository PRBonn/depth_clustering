// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include "./drawable_polygon3d.h"
#include <GL/glut.h>

namespace depth_clustering {

void DrawablePolygon3d::Draw() const {
  if (polygon_.empty()) {
    return;
  }
  glPushMatrix();
  const auto start = polygon_.front();
  glColor3f(color_[0], color_[1], color_[2]);
  glLineWidth(4.0f);
  glBegin(GL_LINE_STRIP);

  // Bottom polygon
  for (const auto& point : polygon_) {
    glVertex3f(point.x(), point.y(), point.z());
  }
  glVertex3f(start.x(), start.y(), start.z());

  // Top polygon
  for (const auto& point : polygon_) {
    glVertex3f(point.x(), point.y(), point.z() + height_);
  }
  glVertex3f(start.x(), start.y(), start.z() + height_);

  glEnd();

  glBegin(GL_LINES);
  // For the Sides of the polygon
  for (const auto& point : polygon_) {
    glVertex3f(point.x(), point.y(), point.z());
    glVertex3f(point.x(), point.y(), point.z() + height_);
  }
  glEnd();
  glPopMatrix();
}

}  // namespace depth_clustering
