// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// GNU-GPL licence that follows one of libQGLViewer.

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
