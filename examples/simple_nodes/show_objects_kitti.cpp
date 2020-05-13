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

#include <stdio.h>

#include <qapplication.h>

#include <string>
#include <thread>

#include "clusterers/image_based_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"

#include "ground_removal/depth_ground_remover.h"
#include "projections/projection_params.h"
#include "qt/drawables/drawable_cloud.h"
#include "qt/drawables/object_painter.h"
#include "utils/cloud.h"
#include "utils/folder_reader.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"

#include "tclap/CmdLine.h"

using std::string;

using namespace depth_clustering;

void ReadData(const Radians& angle_tollerance, const string& in_path,
              Viewer* visualizer) {
  // delay reading for one second to allow GUI to load
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // now load the data
  fprintf(stderr, "INFO: running on kitti data\n");

  int min_cluster_size = 20;
  int max_cluster_size = 100000;

  int smooth_window_size = 9;
  Radians ground_remove_angle = 7_deg;

  auto cloud_reader =
      FolderReader(in_path, ".bin", FolderReader::Order::SORTED);

  auto proj_params_ptr = ProjectionParams::HDL_64();

  auto depth_ground_remover = DepthGroundRemover(
      *proj_params_ptr, ground_remove_angle, smooth_window_size);

  ImageBasedClusterer<LinearImageLabeler<>> clusterer(
      angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

  ObjectPainter object_painter{visualizer,
                               ObjectPainter::OutlineType::kPolygon3d};

  depth_ground_remover.AddClient(&clusterer);
  clusterer.AddClient(&object_painter);

  fprintf(stderr, "INFO: everything initialized\n");

  for (auto path : cloud_reader.GetAllFilePaths()) {
    time_utils::Timer timer;
    auto cloud = ReadKittiCloud(path);
    cloud->InitProjection(*proj_params_ptr);
    visualizer->Clear();
    visualizer->AddDrawable(DrawableCloud::FromCloud(cloud));
    depth_ground_remover.OnNewObjectReceived(*cloud, 0);

    uint max_wait_time = 100;
    auto current_millis = timer.measure(time_utils::Timer::Units::Milli);
    fprintf(stderr, "INFO: It took %lu ms to process.\n", current_millis);
    if (current_millis > max_wait_time) {
      continue;
    }
    auto time_to_wait = max_wait_time - current_millis;
    fprintf(stderr, "INFO: Waiting another %lu ms.\n", time_to_wait);
    std::this_thread::sleep_for(std::chrono::milliseconds(time_to_wait));
  }
}

int main(int argc, char* argv[]) {
  TCLAP::CmdLine cmd(
      "Loads clouds from KITTI data and performs clustering on them.", ' ',
      "1.0");
  TCLAP::ValueArg<int> angle_arg(
      "", "angle",
      "Threshold angle. Below this value, the objects are separated", false, 10,
      "int");
  TCLAP::ValueArg<string> path_to_data_arg(
      "", "path", "Path to folder that stores the data", true, "", "string");

  cmd.add(angle_arg);
  cmd.add(path_to_data_arg);
  cmd.parse(argc, argv);

  Radians angle_tollerance = Radians::FromDegrees(angle_arg.getValue());
  string in_path = path_to_data_arg.getValue();
  fprintf(stderr, "INFO: Reading from: %s \n", in_path.c_str());

  QApplication application(argc, argv);
  // visualizer should be created from a gui thread
  Viewer visualizer;
  visualizer.show();

  // create and run loader thread
  std::thread loader_thread(ReadData, angle_tollerance, in_path, &visualizer);

  // if we close the qt application we will be here
  auto exit_code = application.exec();

  // join thread after the application is dead
  loader_thread.join();
  return exit_code;
}
