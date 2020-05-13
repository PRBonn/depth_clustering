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

#include "./opengl_folder_player.h"

#include <QColor>
#include <QDebug>
#include <QFileDialog>
#include <QImage>
#include <QPixmap>
#include <QUuid>

#include <utils/folder_reader.h>
#include <utils/timer.h>
#include <utils/velodyne_utils.h>

#if PCL_FOUND
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#endif  // PCL_FOUND

#include <vector>

#include "qt/drawables/drawable_cloud.h"
#include "qt/drawables/drawable_cube.h"
#include "qt/drawables/object_painter.h"
#include "qt/utils/utils.h"

#include "qt/widgets/ui_opengl_folder_player.h"

#include "image_labelers/diff_helpers/diff_factory.h"

using depth_clustering::AbstractImageLabeler;
using depth_clustering::AngleDiffPrecomputed;
using depth_clustering::Cloud;
using depth_clustering::DepthGroundRemover;
using depth_clustering::DiffFactory;
using depth_clustering::FolderReader;
using depth_clustering::ImageBasedClusterer;
using depth_clustering::LinearImageLabeler;
using depth_clustering::MatFromDepthPng;
using depth_clustering::ObjectPainter;
using depth_clustering::ProjectionParams;
using depth_clustering::Radians;
using depth_clustering::ReadKittiCloud;
using depth_clustering::ReadKittiCloudTxt;
using depth_clustering::time_utils::Timer;
using std::vector;

OpenGlFolderPlayer::OpenGlFolderPlayer(QWidget *parent)
    : BaseViewerWidget(parent), ui(new Ui::OpenGlFolderPlayer) {
  ui->setupUi(this);
  ui->sldr_navigate_clouds->setEnabled(false);
  ui->spnbx_current_cloud->setEnabled(false);

  _viewer = ui->gl_widget;
  _viewer->installEventFilter(this);
  _viewer->setAutoFillBackground(true);

  connect(ui->btn_open_folder, SIGNAL(released()), this,
          SLOT(onOpenFolderToRead()));
  connect(ui->sldr_navigate_clouds, SIGNAL(valueChanged(int)), this,
          SLOT(onSliderMovedTo(int)));
  connect(ui->btn_play, SIGNAL(released()), this, SLOT(onPlayAllClouds()));

  connect(ui->spnbx_min_cluster_size, SIGNAL(valueChanged(int)), this,
          SLOT(onSegmentationParamUpdate()));
  connect(ui->spnbx_max_cluster_size, SIGNAL(valueChanged(int)), this,
          SLOT(onSegmentationParamUpdate()));
  connect(ui->spnbx_ground_angle, SIGNAL(valueChanged(double)), this,
          SLOT(onSegmentationParamUpdate()));
  connect(ui->spnbx_separation_angle, SIGNAL(valueChanged(double)), this,
          SLOT(onSegmentationParamUpdate()));
  connect(ui->spnbx_smooth_window_size, SIGNAL(valueChanged(int)), this,
          SLOT(onSegmentationParamUpdate()));
  connect(ui->radio_show_segmentation, SIGNAL(toggled(bool)), this,
          SLOT(onSegmentationParamUpdate()));
  connect(ui->radio_show_angles, SIGNAL(toggled(bool)), this,
          SLOT(onSegmentationParamUpdate()));
  connect(ui->cmb_diff_type, SIGNAL(activated(int)), this,
          SLOT(onSegmentationParamUpdate()));

  // setup viewer
  _cloud.reset(new Cloud);

  _proj_params = ProjectionParams::HDL_64();

  ui->gfx_projection_view->setViewportUpdateMode(
      QGraphicsView::BoundingRectViewportUpdate);
  ui->gfx_projection_view->setCacheMode(QGraphicsView::CacheBackground);
  ui->gfx_projection_view->setRenderHints(QPainter::Antialiasing |
                                          QPainter::SmoothPixmapTransform);

  ui->gfx_labels->setViewportUpdateMode(
      QGraphicsView::BoundingRectViewportUpdate);
  ui->gfx_labels->setCacheMode(QGraphicsView::CacheBackground);
  ui->gfx_labels->setRenderHints(QPainter::Antialiasing |
                                 QPainter::SmoothPixmapTransform);

  _painter.reset(
      new ObjectPainter{_viewer, ObjectPainter::OutlineType::kPolygon3d});
  this->onSegmentationParamUpdate();
}

void OpenGlFolderPlayer::onPlayAllClouds() {
  for (int i = ui->sldr_navigate_clouds->minimum();
       i < ui->sldr_navigate_clouds->maximum(); ++i) {
    ui->sldr_navigate_clouds->setValue(i);
    ui->gl_widget->update();
    QApplication::processEvents();
  }
  qDebug() << "All clouds shown!";
}

void OpenGlFolderPlayer::OnNewObjectReceived(const cv::Mat &image, int) {
  QImage qimage;
  fprintf(stderr, "[INFO] Received Mat with type: %d\n", image.type());
  switch (image.type()) {
    case cv::DataType<float>::type: {
      // we have received a depth image
      fprintf(stderr, "[INFO] received depth.\n");
      DiffFactory::DiffType diff_type = DiffFactory::DiffType::NONE;
      switch (ui->cmb_diff_type->currentIndex()) {
        case 0: {
          fprintf(stderr, "Using DiffFactory::DiffType::ANGLES\n");
          diff_type = DiffFactory::DiffType::ANGLES;
          break;
        }
        case 1: {
          fprintf(stderr, "Using DiffFactory::DiffType::ANGLES_PRECOMPUTED\n");
          diff_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;
          break;
        }
        case 2: {
          fprintf(stderr, "Using DiffFactory::DiffType::LINE_DIST\n");
          diff_type = DiffFactory::DiffType::LINE_DIST;
          break;
        }
        case 3: {
          fprintf(stderr,
                  "Using DiffFactory::DiffType::LINE_DIST_PRECOMPUTED\n");
          diff_type = DiffFactory::DiffType::LINE_DIST_PRECOMPUTED;
          break;
        }
        default: {
          fprintf(stderr, "Using DiffFactory::DiffType::SIMPLE\n");
          diff_type = DiffFactory::DiffType::SIMPLE;
        }
      }
      auto diff_helper_ptr =
          DiffFactory::Build(diff_type, &image, _proj_params.get());
      qimage = MatToQImage(diff_helper_ptr->Visualize());
      break;
    }
    case cv::DataType<uint16_t>::type: {
      // we have received labels
      fprintf(stderr, "[INFO] received labels.\n");
      qimage = MatToQImage(AbstractImageLabeler::LabelsToColor(image));
      break;
    }
    default: {
      fprintf(stderr, "ERROR: unknown type Mat received.\n");
      return;
    }
  }
  _scene_labels.reset(new QGraphicsScene);
  _scene_labels->addPixmap(QPixmap::fromImage(qimage));
  ui->gfx_labels->setScene(_scene_labels.get());
  ui->gfx_labels->fitInView(_scene_labels->itemsBoundingRect());
}

void OpenGlFolderPlayer::onSegmentationParamUpdate() {
  // setup segmentation
  fprintf(stderr, "Info: update segmentation parameters\n");
  int smooth_window_size = ui->spnbx_smooth_window_size->value();
  Radians ground_remove_angle =
      Radians::FromDegrees(ui->spnbx_ground_angle->value());
  Radians angle_tollerance =
      Radians::FromDegrees(ui->spnbx_separation_angle->value());
  int min_cluster_size = ui->spnbx_min_cluster_size->value();
  int max_cluster_size = ui->spnbx_max_cluster_size->value();

  // create objects

  DiffFactory::DiffType diff_type = DiffFactory::DiffType::NONE;
  switch (ui->cmb_diff_type->currentIndex()) {
    case 0: {
      fprintf(stderr, "Using DiffFactory::DiffType::ANGLES\n");
      diff_type = DiffFactory::DiffType::ANGLES;
      break;
    }
    case 1: {
      fprintf(stderr, "Using DiffFactory::DiffType::ANGLES_PRECOMPUTED\n");
      diff_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;
      break;
    }
    case 2: {
      fprintf(stderr, "Using DiffFactory::DiffType::LINE_DIST\n");
      diff_type = DiffFactory::DiffType::LINE_DIST;
      break;
    }
    case 3: {
      fprintf(stderr, "Using DiffFactory::DiffType::LINE_DIST_PRECOMPUTED\n");
      diff_type = DiffFactory::DiffType::LINE_DIST_PRECOMPUTED;
      break;
    }
    default: {
      fprintf(stderr, "Using DiffFactory::DiffType::SIMPLE\n");
      diff_type = DiffFactory::DiffType::SIMPLE;
    }
  }
  _clusterer.reset(new ImageBasedClusterer<LinearImageLabeler<>>(
      angle_tollerance, min_cluster_size, max_cluster_size));
  ui->spnbx_line_dist_threshold->setValue(angle_tollerance.val());
  _clusterer->SetDiffType(diff_type);
  _ground_rem.reset(new DepthGroundRemover(*_proj_params, ground_remove_angle,
                                           smooth_window_size));
  // configure wires
  _ground_rem->AddClient(_clusterer.get());
  _clusterer->AddClient(_painter.get());
  if (ui->radio_show_segmentation->isChecked()) {
    fprintf(stderr, "Info: ready to receive labels\n");
    _clusterer->SetLabelImageClient(this);
  } else {
    _scene_labels.reset();
  }
  this->onSliderMovedTo(ui->sldr_navigate_clouds->value());
}

void OpenGlFolderPlayer::onSliderMovedTo(int cloud_number) {
  if (_file_names.empty()) {
    return;
  }
  fprintf(stderr, "slider moved to: %d\n", cloud_number);
  fprintf(stderr, "loading cloud from: %s\n",
          _file_names[cloud_number].c_str());
  Timer timer;
  const auto &file_name = _file_names[cloud_number];
  _cloud = CloudFromFile(file_name, *_proj_params);
  fprintf(stderr, "[TIMER]: load cloud in %lu microsecs\n",
          timer.measure(Timer::Units::Micro));
  _current_full_depth_image = _cloud->projection_ptr()->depth_image();

  ui->lbl_cloud_name->setText(QString::fromStdString(file_name));

  timer.start();
  QImage qimage = MatToQImage(_current_full_depth_image);
  _scene.reset(new QGraphicsScene);
  _scene->addPixmap(QPixmap::fromImage(qimage));
  ui->gfx_projection_view->setScene(_scene.get());
  ui->gfx_projection_view->fitInView(_scene->itemsBoundingRect());
  fprintf(stderr, "[TIMER]: depth image set to gui in %lu microsecs\n",
          timer.measure(Timer::Units::Micro));
  if (ui->radio_show_angles->isChecked()) {
    this->OnNewObjectReceived(_current_full_depth_image);
  }
  fprintf(stderr, "[TIMER]: angles shown in %lu microsecs\n",
          timer.measure(Timer::Units::Micro));

  _viewer->Clear();
  _viewer->AddDrawable(DrawableCloud::FromCloud(_cloud));
  _viewer->update();

  fprintf(stderr, "[TIMER]: add cloud to gui in %lu microsecs\n",
          timer.measure(Timer::Units::Micro));

  // label cloud and show labels
  _ground_rem->OnNewObjectReceived(*_cloud, 0);

  fprintf(stderr, "[TIMER]: full segmentation took %lu milliseconds\n",
          timer.measure(Timer::Units::Milli));
}

void OpenGlFolderPlayer::onOpenFolderToRead() {
  // create a dialog here
  QString folder_name = QFileDialog::getExistingDirectory(this);
  qDebug() << "Picked path:" << folder_name;

  _file_names.clear();
  FolderReader::Order order = FolderReader::Order::SORTED;
  FolderReader cloud_reader(folder_name.toStdString(),
                            ui->cmb_extension->currentText().toStdString(),
                            order);
  _file_names = cloud_reader.GetAllFilePaths();
  if (_file_names.empty()) {
    return;
  }

  // update the slider
  ui->sldr_navigate_clouds->setMaximum(_file_names.size() - 1);
  ui->spnbx_current_cloud->setMaximum(_file_names.size() - 1);

  // set current value
  ui->sldr_navigate_clouds->setValue(1);
  ui->sldr_navigate_clouds->setEnabled(true);
  ui->spnbx_current_cloud->setEnabled(true);

  // focus on the cloud
  _viewer->update();
}

void OpenGlFolderPlayer::keyPressEvent(QKeyEvent *event) {
  switch (event->key()) {
    case Qt::Key_Right:
      ui->spnbx_current_cloud->setValue(ui->spnbx_current_cloud->value() + 1);
      break;
    case Qt::Key_Left:
      ui->spnbx_current_cloud->setValue(ui->spnbx_current_cloud->value() - 1);
      break;
  }
}

OpenGlFolderPlayer::~OpenGlFolderPlayer() {}
