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

#include "ros_bridge/cloud_odom_ros_subscriber.h"
#include <eigen_conversions/eigen_msg.h>

#include <vector>
#include <string>
#include <algorithm>

#include "utils/pose.h"

namespace depth_clustering {

using ros::NodeHandle;
using message_filters::Subscriber;
using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;
using nav_msgs::Odometry;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

using std::vector;
using std::string;
using std::map;

template <class T>
T BytesTo(const vector<uint8_t>& data, uint32_t start_idx) {
  const size_t kNumberOfBytes = sizeof(T);
  uint8_t byte_array[kNumberOfBytes];
  // forward bit order (it is a HACK. We do not account for bigendianes)
  for (size_t i = 0; i < kNumberOfBytes; ++i) {
    byte_array[i] = data[start_idx + i];
  }
  T result;
  std::copy(reinterpret_cast<const uint8_t*>(&byte_array[0]),
            reinterpret_cast<const uint8_t*>(&byte_array[kNumberOfBytes]),
            reinterpret_cast<uint8_t*>(&result));
  return result;
}

void PrintMsgStats(const sensor_msgs::PointCloud2ConstPtr& msg) {
  fprintf(stderr, "<<<<<<<<<<<<<<< new cloud >>>>>>>>>>>>>>>\n");
  fprintf(stderr, "received msg   %d\n", msg->header.seq);
  fprintf(stderr, "height:        %d\n", msg->height);
  fprintf(stderr, "width:         %d\n", msg->width);
  fprintf(stderr, "num of fields: %lu\n", msg->fields.size());
  fprintf(stderr, "fields of each point:\n");
  for (auto const& pointField : msg->fields) {
    fprintf(stderr, "\tname:     %s\n", pointField.name.c_str());
    fprintf(stderr, "\toffset:   %d\n", pointField.offset);
    fprintf(stderr, "\tdatatype: %d\n", pointField.datatype);
    fprintf(stderr, "\tcount:    %d\n", pointField.count);
    fprintf(stderr, "\n");
  }
  fprintf(stderr, "is bigendian:  %s\n", msg->is_bigendian ? "true" : "false");
  fprintf(stderr, "point step:    %d\n", msg->point_step);
  fprintf(stderr, "row step:      %d\n", msg->row_step);
  fprintf(stderr, "data size:     %lu\n", msg->data.size() * sizeof(msg->data));
  fprintf(stderr, "is dense:      %s\n", msg->is_dense ? "true" : "false");
  fprintf(stderr, "=========================================\n");
}

CloudOdomRosSubscriber::CloudOdomRosSubscriber(NodeHandle* node_handle,
                                               const ProjectionParams& params,
                                               const string& topic_clouds,
                                               const string& topic_odom)
    : AbstractSender{SenderType::STREAMER}, _params{params} {
  _node_handle = node_handle;
  _topic_clouds = topic_clouds;
  _topic_odom = topic_odom;
  _msg_queue_size = 100;

  _subscriber_clouds = nullptr;
  _subscriber_odom = nullptr;
  _sync = nullptr;
}

void CloudOdomRosSubscriber::StartListeningToRos() {
  if (!_topic_odom.empty()) {
    _subscriber_clouds = new Subscriber<PointCloud2>(
        *_node_handle, _topic_clouds, _msg_queue_size);
    _subscriber_odom =
        new Subscriber<Odometry>(*_node_handle, _topic_odom, _msg_queue_size);
    _sync = new Synchronizer<ApproximateTimePolicy>(
        ApproximateTimePolicy(100), *_subscriber_clouds, *_subscriber_odom);
    _sync->registerCallback(
        boost::bind(&CloudOdomRosSubscriber::Callback, this, _1, _2));
  } else {
    _subscriber_clouds = new Subscriber<PointCloud2>(
        *_node_handle, _topic_clouds, _msg_queue_size);
    _subscriber_clouds->registerCallback(
        &CloudOdomRosSubscriber::CallbackVelodyne, this);
  }
}

void CloudOdomRosSubscriber::Callback(const PointCloud2::ConstPtr& msg_cloud,
                                      const Odometry::ConstPtr& msg_odom) {
  // PrintMsgStats(msg_cloud);
  Cloud::Ptr cloud_ptr = RosCloudToCloud(msg_cloud);
  cloud_ptr->SetPose(RosOdomToPose(msg_odom));
  cloud_ptr->InitProjection(_params);
  ShareDataWithAllClients(*cloud_ptr);
}

void CloudOdomRosSubscriber::CallbackVelodyne(
    const PointCloud2::ConstPtr& msg_cloud) {
  // PrintMsgStats(msg_cloud);
  Cloud::Ptr cloud_ptr = RosCloudToCloud(msg_cloud);
  cloud_ptr->InitProjection(_params);
  ShareDataWithAllClients(*cloud_ptr);
}

Pose CloudOdomRosSubscriber::RosOdomToPose(const Odometry::ConstPtr& msg) {
  Pose pose;
  // we want float, so some casting is needed
  Eigen::Affine3d pose_double;
  tf::poseMsgToEigen(msg->pose.pose, pose_double);
  pose = pose_double.cast<float>();
  return pose;
}

Cloud::Ptr CloudOdomRosSubscriber::RosCloudToCloud(
    const PointCloud2::ConstPtr& msg) {
  uint32_t x_offset = msg->fields[0].offset;
  uint32_t y_offset = msg->fields[1].offset;
  uint32_t z_offset = msg->fields[2].offset;
  uint32_t ring_offset = msg->fields[4].offset;

  Cloud cloud;
  for (uint32_t point_start_byte = 0, counter = 0;
       point_start_byte < msg->data.size();
       point_start_byte += msg->point_step, ++counter) {
    RichPoint point;
    point.x() = BytesTo<float>(msg->data, point_start_byte + x_offset);
    point.y() = BytesTo<float>(msg->data, point_start_byte + y_offset);
    point.z() = BytesTo<float>(msg->data, point_start_byte + z_offset);
    point.ring() = BytesTo<uint16_t>(msg->data, point_start_byte + ring_offset);
    // point.z *= -1;  // hack
    cloud.push_back(point);
  }

  return make_shared<Cloud>(cloud);
}

}  // namespace depth_clustering
