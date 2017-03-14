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

#include <stdio.h>

#include <pcl/io/hdl_grabber.h>
#include <qapplication.h>

#include <string>
#include <thread>

#include "clusterers/image_based_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"

#include "ground_removal/depth_ground_remover.h"
#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/folder_reader.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"
#include "visualization/visualizer.h"

#include "tclap/CmdLine.h"

using std::string;
using std::to_string;

using PclCloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudConstPtr = typename PclCloud::ConstPtr;

using namespace depth_clustering;

struct PcapBridge {
  PcapBridge(const Radians& angle_tollerance, const string& calib_file,
             const string& pcap_file, Visualizer* visualizer,
             ProjectionParams* proj_params_ptr)
      : visualizer(visualizer),
        proj_params_ptr(proj_params_ptr),
        grabber(calib_file, pcap_file),
        clusterer(angle_tollerance, 100, 100000),
        depth_ground_remover(*proj_params_ptr, 8_deg, 5) {
    clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);
    depth_ground_remover.AddClient(&clusterer);
    clusterer.AddClient(visualizer->object_clouds_client());
  }

  void CloudCallback(const CloudConstPtr& pcl_cloud_ptr) {
    // do smth here
    fprintf(stderr, "cloud size: %lu\n", pcl_cloud_ptr->size());
    auto cloud_ptr = Cloud::FromPcl(*pcl_cloud_ptr);
    cloud_ptr->InitProjection(*proj_params_ptr);
    visualizer->OnNewObjectReceived(*cloud_ptr, 0);
    depth_ground_remover.OnNewObjectReceived(*cloud_ptr, 0);
  }

  void ReadData() {
    // delay reading for one second to allow GUI to load
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // now load the data
    fprintf(stderr, "[INFO]: running on pcap file\n");

    boost::function<void(const CloudConstPtr&)> cloud_cb =
        boost::bind(&PcapBridge::CloudCallback, this, _1);
    boost::signals2::connection cloud_connection =
        grabber.registerCallback(cloud_cb);
    grabber.start();
    while (grabber.isRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    grabber.stop();
    cloud_connection.disconnect();
    fprintf(stderr, "[INFO]: grabber stopped\n");
  }

  Visualizer* visualizer;
  ProjectionParams* proj_params_ptr;
  pcl::HDLGrabber grabber;
  ImageBasedClusterer<LinearImageLabeler<>> clusterer;
  DepthGroundRemover depth_ground_remover;
};

int main(int argc, char* argv[]) {
  TCLAP::CmdLine cmd("Visualize segmentation based on a pcap file.", ' ',
                     "1.0");
  TCLAP::ValueArg<int> angle_arg(
      "", "angle",
      "Threshold angle. Below this value, the objects are separated", false, 10,
      "int");
  TCLAP::ValueArg<string> path_to_pcap_file(
      "p", "pcap_file_path", "Path to a pcap file", true, "", "string");
  TCLAP::ValueArg<string> path_to_calibraion_file(
      "c", "calib_file_path", "Path to a calibration file", true, "", "string");

  cmd.add(angle_arg);
  cmd.add(path_to_pcap_file);
  cmd.add(path_to_calibraion_file);
  cmd.parse(argc, argv);

  Radians angle_tollerance = Radians::FromDegrees(angle_arg.getValue());
  string pcap_path = path_to_pcap_file.getValue();
  string calib_path = path_to_calibraion_file.getValue();
  fprintf(stderr, "INFO: Reading from: %s; calibration file: %s \n",
          pcap_path.c_str(), calib_path.c_str());

  QApplication application(argc, argv);
  // visualizer should be created from a gui thread
  Visualizer visualizer;
  visualizer.show();

  // create and run loader thread
  auto proj_params_ptr = ProjectionParams::HDL_32();
  PcapBridge pcap_bridge(angle_tollerance, calib_path, pcap_path, &visualizer,
                         proj_params_ptr.get());
  std::thread loader_thread(&PcapBridge::ReadData, &pcap_bridge);

  // if we close the qt application we will be here
  auto exit_code = application.exec();

  // join thread after the application is dead
  loader_thread.join();
  return exit_code;
}
