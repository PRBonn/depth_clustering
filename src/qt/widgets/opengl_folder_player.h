// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_QT_PCL_FOLDER_PLAYER_H_
#define SRC_QT_PCL_FOLDER_PLAYER_H_

#include <QGraphicsScene>
#include <QKeyEvent>
#include <QWidget>

#include <opencv2/opencv.hpp>

#include "qt/drawables/object_painter.h"
#include "qt/viewer/viewer.h"
#include "qt/widgets/base_viewer_widget.h"
#include <memory>
#include <string>
#include <vector>

#include "clusterers/image_based_clusterer.h"
#include "communication/abstract_client.h"
#include "ground_removal/depth_ground_remover.h"
#include "projections/cloud_projection.h"
#include "projections/spherical_projection.h"
#include "utils/cloud.h"

namespace Ui {
class OpenGlFolderPlayer;
}

class OpenGlFolderPlayer : public BaseViewerWidget,
                           public depth_clustering::AbstractClient<cv::Mat> {
  Q_OBJECT

 public:
  explicit OpenGlFolderPlayer(QWidget *parent = 0);
  void OnNewObjectReceived(const cv::Mat &labels, int client_id = 0) override;
  virtual ~OpenGlFolderPlayer();

 protected:
  void keyPressEvent(QKeyEvent *event) override;

 private slots:
  void onOpenFolderToRead();
  void onPlayAllClouds();
  void onSegmentationParamUpdate();
  void onSliderMovedTo(int cloud_number);

 private:
  // we cannot allocate anything now as this is an incomplete type
  std::unique_ptr<Ui::OpenGlFolderPlayer> ui;

  std::unique_ptr<QGraphicsScene> _scene = nullptr;
  std::unique_ptr<QGraphicsScene> _scene_labels = nullptr;
  std::unique_ptr<depth_clustering::ProjectionParams> _proj_params = nullptr;

  std::unique_ptr<depth_clustering::ImageBasedClusterer<
      depth_clustering::LinearImageLabeler<>>>
      _clusterer = nullptr;
  std::unique_ptr<depth_clustering::DepthGroundRemover> _ground_rem = nullptr;

  std::unique_ptr<ObjectPainter> _painter = nullptr;

  cv::Mat _current_full_depth_image;
  depth_clustering::Cloud::Ptr _cloud;
  std::vector<std::string> _file_names;

  int _current_coloring_mode = 0;

  std::vector<std::string> _current_object_labels;
};

#endif  // SRC_QT_PCL_FOLDER_PLAYER_H_
