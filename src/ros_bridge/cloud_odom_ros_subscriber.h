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

#ifndef SRC_ROS_BRIDGE_CLOUD_ODOM_ROS_SUBSCRIBER_H_
#define SRC_ROS_BRIDGE_CLOUD_ODOM_ROS_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string>
#include <map>

#include "communication/abstract_sender.h"
#include "utils/pose.h"
#include "utils/cloud.h"
#include "utils/useful_typedefs.h"

namespace depth_clustering {

/**
 * @brief      Class for cloud odom ros subscriber.
 */
class CloudOdomRosSubscriber : public AbstractSender<Cloud> {
  using PointCloudT = sensor_msgs::PointCloud2;
  using OdometryT = nav_msgs::Odometry;
  using ApproximateTimePolicy =
      message_filters::sync_policies::ApproximateTime<PointCloudT, OdometryT>;

 public:
  CloudOdomRosSubscriber(ros::NodeHandle* node_handle,
                         const ProjectionParams& params,
                         const std::string& topic_clouds,
                         const std::string& topic_odom = "");
  virtual ~CloudOdomRosSubscriber() {
    delete _subscriber_odom;
    delete _subscriber_clouds;
    delete _sync;
  }

  /**
   * @brief      Get synchronized odometry and cloud
   *
   * @param[in]  msg_cloud  The message cloud
   * @param[in]  msg_odom   The message odom
   */
  void Callback(const PointCloudT::ConstPtr& msg_cloud,
                const OdometryT::ConstPtr& msg_odom);

  /**
   * @brief      Get point cloud from ROS
   *
   * @param[in]  msg_cloud  The message cloud
   */
  void CallbackVelodyne(const sensor_msgs::PointCloud2::ConstPtr& msg_cloud);

  /**
   * @brief      Starts listening to ros.
   */
  void StartListeningToRos();

 protected:
  Pose RosOdomToPose(const OdometryT::ConstPtr& msg);
  Cloud::Ptr RosCloudToCloud(const PointCloudT::ConstPtr& msg);

  ros::NodeHandle* _node_handle;

  message_filters::Subscriber<PointCloudT>* _subscriber_clouds;
  message_filters::Subscriber<OdometryT>* _subscriber_odom;
  message_filters::Synchronizer<ApproximateTimePolicy>* _sync;
  std::string _topic_clouds;
  std::string _topic_odom;

  ProjectionParams _params;

  int _msg_queue_size;
};

}  // namespace depth_clustering

#endif  // SRC_ROS_BRIDGE_CLOUD_ODOM_ROS_SUBSCRIBER_H_
