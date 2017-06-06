// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#pragma once

#include <QPushButton>
#include <QTabWidget>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>
#include <QtGui>

#include <string>

class TabbedWidget : public QWidget {
  Q_OBJECT

 public:
  explicit TabbedWidget(QWidget* parent = 0);
  void AddTab(QWidget* tab, const QString& name);
  ~TabbedWidget() {}

 private:
  QTabWidget* _tabs;
};
