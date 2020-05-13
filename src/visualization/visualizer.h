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
