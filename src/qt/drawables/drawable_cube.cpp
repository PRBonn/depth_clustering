// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include "./drawable_cube.h"
#include <GL/glut.h>

void DrawableCube::Draw() const {
  if (_scale.z() < 0.3) { return; }
  glPushMatrix();
  glTranslatef(_center.x(), _center.y(), _center.z());
  glScalef(_scale.x(), _scale.y(), _scale.z());
  if (_scale.prod() > 20.0f || _scale.x() > 5.0f || _scale.y() > 5.0f ||
      _scale.z() > 5.0f) {
    glColor3f(0.3f, 0.3f, 0.3f);
  } else {
    glColor3f(_color[0], _color[1], _color[2]);
  }
  glLineWidth(4.0f);
  glBegin(GL_LINE_STRIP);

  // Bottom of Box
  glVertex3f(-0.5, -0.5, -0.5);
  glVertex3f(-0.5, -0.5, 0.5);
  glVertex3f(0.5, -0.5, 0.5);
  glVertex3f(0.5, -0.5, -0.5);
  glVertex3f(-0.5, -0.5, -0.5);

  // Top of Box
  glVertex3f(-0.5, 0.5, -0.5);
  glVertex3f(-0.5, 0.5, 0.5);
  glVertex3f(0.5, 0.5, 0.5);
  glVertex3f(0.5, 0.5, -0.5);
  glVertex3f(-0.5, 0.5, -0.5);

  glEnd();

  glBegin(GL_LINES);
  // For the Sides of the Box

  glVertex3f(-0.5, 0.5, -0.5);
  glVertex3f(-0.5, -0.5, -0.5);

  glVertex3f(-0.5, -0.5, 0.5);
  glVertex3f(-0.5, 0.5, 0.5);

  glVertex3f(0.5, -0.5, 0.5);
  glVertex3f(0.5, 0.5, 0.5);

  glVertex3f(0.5, -0.5, -0.5);
  glVertex3f(0.5, 0.5, -0.5);

  glEnd();
  glPopMatrix();
}
