// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include "qt/widgets/base_viewer_widget.h"

#include <QKeyEvent>

BaseViewerWidget::BaseViewerWidget(QWidget *parent) : QWidget(parent) {}

bool BaseViewerWidget::eventFilter(QObject *object, QEvent *event) {
  if (event->type() == QEvent::KeyPress) {
    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
    if (keyEvent->key() == Qt::Key_Right || keyEvent->key() == Qt::Key_Left) {
      keyPressEvent(keyEvent);
      return true;
    } else {
      return false;
    }
  }
  return false;
}
