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

#include <ros/ros.h>

#include <qapplication.h>

#include <string>

#include "ros_bridge/cloud_odom_ros_subscriber.h"

#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "projections/ring_projection.h"
#include "projections/spherical_projection.h"
#include "utils/radians.h"
#include "visualization/cloud_saver.h"
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
