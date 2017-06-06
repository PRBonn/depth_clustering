// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#pragma once

#include <QWidget>
#include "qt/viewer/viewer.h"

class BaseViewerWidget : public QWidget {
  Q_OBJECT

 public:
  explicit BaseViewerWidget(QWidget *parent);
  ~BaseViewerWidget() override {}

 protected:
  bool eventFilter(QObject *obj, QEvent *event) override;

  Viewer *_viewer = nullptr;
};
