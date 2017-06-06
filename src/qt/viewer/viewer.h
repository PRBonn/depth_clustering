// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_QT_VIEWER_VIEWER_H_
#define SRC_QT_VIEWER_VIEWER_H_

#include <QGLViewer/qglviewer.h>

#include <mutex>
#include <vector>

#include "qt/drawables/drawable.h"

class Viewer : public QGLViewer {
 public:
  explicit Viewer(QWidget* parent = 0) : QGLViewer(parent) {}
  void AddDrawable(Drawable::Ptr drawable);
  void Clear();
  ~Viewer() override {}

 protected:
  void draw() override;
  void init() override;

 private:
  std::vector<Drawable::Ptr> _drawables;
  mutable std::mutex _cloud_mutex;
};

#endif  // SRC_QT_VIEWER_VIEWER_H_
