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

#include "./viewer.h"

using std::lock_guard;
using std::mutex;

void Viewer::AddDrawable(Drawable::Ptr drawable) {
  lock_guard<mutex> guard(_cloud_mutex);
  _drawables.push_back(drawable);
}

void Viewer::Clear() {
  lock_guard<mutex> guard(_cloud_mutex);
  _drawables.clear();
}

void Viewer::draw() {
  lock_guard<mutex> guard(_cloud_mutex);
  for (auto& drawable : _drawables) {
    drawable->Draw();
  }
}

void Viewer::init() {
  setSceneRadius(100.0);
  setBackgroundColor(QColor(0, 0, 0));
  camera()->showEntireScene();
  glDisable(GL_LIGHTING);
}
