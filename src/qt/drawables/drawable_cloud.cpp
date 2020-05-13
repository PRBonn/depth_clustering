// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

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
