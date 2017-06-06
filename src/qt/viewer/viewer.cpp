// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include "./viewer.h"

using std::mutex;
using std::lock_guard;

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
