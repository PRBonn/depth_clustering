#include <QApplication>
#include <QMainWindow>

#include "qt/widgets/opengl_folder_player.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  OpenGlFolderPlayer folder_player_widget;
  folder_player_widget.show();

  return a.exec();
}
