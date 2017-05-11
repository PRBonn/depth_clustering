// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SRC_VISUALIZATION_VISUALIZER_H_
#define SRC_VISUALIZATION_VISUALIZER_H_

#include <QGLViewer/qglviewer.h>

#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "communication/abstract_client.h"
#include "utils/cloud.h"
#include "utils/useful_typedefs.h"

namespace depth_clustering {

class IUpdateListener {
 public:
  virtual void onUpdate() = 0;
};

class ObjectPtrStorer
    : public AbstractClient<std::unordered_map<uint16_t, Cloud>> {
 public:
  ObjectPtrStorer() : AbstractClient<std::unordered_map<uint16_t, Cloud>>() {}

  void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds,
                           const int id) override;

  void SetUpdateListener(IUpdateListener* update_listener) {
    _update_listener = update_listener;
  }

  virtual ~ObjectPtrStorer() {}

  std::unordered_map<uint16_t, Cloud> object_clouds() const;

 private:
  std::unordered_map<uint16_t, Cloud> _obj_clouds;
  IUpdateListener* _update_listener;
  mutable std::mutex _cluster_mutex;
};

/**
 * @brief      An OpenGl visualizer that shows data that is subscribes to.
 */
class Visualizer : public QGLViewer,
                   public AbstractClient<Cloud>,
                   public IUpdateListener {
 public:
  explicit Visualizer(QWidget* parent = 0);
  virtual ~Visualizer();

  void OnNewObjectReceived(const Cloud& cloud, const int id) override;

  void onUpdate() override;

  ObjectPtrStorer* object_clouds_client() { return &_cloud_obj_storer; }

 protected:
  void draw() override;
  void init() override;

 private:
  void DrawCloud(const Cloud& cloud);
  void DrawCube(const Eigen::Vector3f& center, const Eigen::Vector3f& scale);

  bool _updated;
  ObjectPtrStorer _cloud_obj_storer;
  Cloud _cloud;
  mutable std::mutex _cloud_mutex;
};

}  // namespace depth_clustering

#endif  // SRC_VISUALIZATION_VISUALIZER_H_
