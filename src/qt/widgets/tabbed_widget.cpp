// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include "./tabbed_widget.h"

TabbedWidget::TabbedWidget(QWidget *parent) : QWidget(parent) {
  _tabs = new QTabWidget;
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(_tabs);

  setLayout(layout);
}

void TabbedWidget::AddTab(QWidget *tab, const QString &name) {
  if (_tabs) {
    _tabs->addTab(tab, name);
  }
}
