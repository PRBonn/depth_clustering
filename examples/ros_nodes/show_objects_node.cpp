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

#include <ros/ros.h>

#include <qapplication.h>

#include <string>

#include "ros_bridge/cloud_odom_ros_subscriber.h"

#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "projections/ring_projection.h"
#include "projections/spherical_projection.h"
#include "utils/radians.h"
#include "visualization/visualizer.h"

#include "tclap/CmdLine.h"

using std::string;

using namespace depth_clustering;

using ClustererT = ImageBasedClusterer<LinearImageLabeler<>>;

int main(int argc, char* argv[]) {
  TCLAP::CmdLine cmd(
      "Subscribe to /velodyne_points topic and show clustering on the data.",
      ' ', "1.0");
  TCLAP::ValueArg<int> angle_arg(
      "", "angle",
      "Threshold angle. Below this value, the objects are separated", false, 10,
      "int");
  TCLAP::ValueArg<int> num_beams_arg(
      "", "num_beams", "Num of vertical beams in laser. One of: [16, 32, 64].",
      true, 0, "int");

  cmd.add(angle_arg);
  cmd.add(num_beams_arg);
  cmd.parse(argc, argv);

  Radians angle_tollerance = Radians::FromDegrees(angle_arg.getValue());

  std::unique_ptr<ProjectionParams> proj_params_ptr = nullptr;
  switch (num_beams_arg.getValue()) {
    case 16:
      proj_params_ptr = ProjectionParams::VLP_16();
      break;
    case 32:
      proj_params_ptr = ProjectionParams::HDL_32();
      break;
    case 64:
      proj_params_ptr = ProjectionParams::HDL_64();
      break;
  }
  if (!proj_params_ptr) {
    fprintf(stderr,
            "ERROR: wrong number of beams: %d. Should be in [16, 32, 64].\n",
            num_beams_arg.getValue());
    exit(1);
  }

  QApplication application(argc, argv);

  ros::init(argc, argv, "show_objects_node");
  ros::NodeHandle nh;

  string topic_clouds = "/velodyne_points";

  CloudOdomRosSubscriber subscriber(&nh, *proj_params_ptr, topic_clouds);
  Visualizer visualizer;
  visualizer.show();

  int min_cluster_size = 20;
  int max_cluster_size = 100000;

  int smooth_window_size = 7;
  Radians ground_remove_angle = 7_deg;

  auto depth_ground_remover = DepthGroundRemover(
      *proj_params_ptr, ground_remove_angle, smooth_window_size);

  ClustererT clusterer(angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

  subscriber.AddClient(&depth_ground_remover);
  depth_ground_remover.AddClient(&clusterer);
  clusterer.AddClient(visualizer.object_clouds_client());
  subscriber.AddClient(&visualizer);

  fprintf(stderr, "INFO: Running with angle tollerance: %f degrees\n",
          angle_tollerance.ToDegrees());

  subscriber.StartListeningToRos();
  ros::AsyncSpinner spinner(1);
  spinner.start();

  auto exit_code = application.exec();

  // if we close application, still wait for ros to shutdown
  ros::waitForShutdown();
  return exit_code;
}
