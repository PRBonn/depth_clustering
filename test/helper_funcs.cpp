// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include "./helper_funcs.h"
#include <math.h>

#include "utils/radians.h"
#include "utils/useful_typedefs.h"

using depth_clustering::Pose;
using depth_clustering::Cloud;
using depth_clustering::Radians;
using depth_clustering::RichPoint;
using depth_clustering::make_shared;

Cloud::Ptr CreateWallLikeCloud(double max_val, double step, double mult) {
  Cloud::Ptr cloud(new Cloud);
  for (double x = 0; x < max_val; x += step) {
    for (double z = 0; z < max_val; z += step) {
      RichPoint point(x, mult * sin(mult * x), z);
      cloud->push_back(point);
    }
  }
  return cloud;
}

Cloud::Ptr CreateMovedCloud(double max_val, double step, double mult,
                            double dist) {
  Cloud::Ptr cloud(new Cloud);
  for (double x = 0; x < max_val; x += step) {
    for (double z = 0; z < max_val; z += step) {
      RichPoint point;
      point.x() = x;
      point.y() = mult * sin(mult * x);
      point.z() = z;
      cloud->push_back(point);
    }
  }
  return MoveAlongY(cloud, dist);
}

Cloud::Ptr MoveAlongY(Cloud::Ptr cloud, double dist) {
  // translate by 5 meters in x;
  auto transform = Pose(0, dist, 0);
  Cloud::Ptr moved_cloud = cloud->Transform(transform);
  return moved_cloud;
}

Cloud::Ptr RotateAroundZ(Cloud::Ptr cloud, Radians angle) {
  // rotate around z axis
  auto transform = Pose(0, 0, angle.val());
  Cloud::Ptr rotated_cloud = cloud->Transform(transform);
  return rotated_cloud;
}

Cloud::Ptr CreateWallLikeCloudLabeled(double max_val, double step,
                                      uint32_t max_label, double mult) {
  Cloud cloud;
  uint32_t label_counter = 0;
  for (double x = 0; x < max_val; x += step) {
    label_counter = 0;
    for (double z = 0; z < max_val; z += step) {
      RichPoint point;
      point.x() = x;
      point.y() = mult * sin(mult * x);
      point.z() = z;
      point.ring() = label_counter;
      if (++label_counter > max_label) {
        break;
      }
      cloud.push_back(point);
    }
  }
  return make_shared<Cloud>(cloud);
}

Cloud::Ptr CreateCyllinderCloud(float radius, int num_beams_h, int num_beams_v,
                                float height_margin, Radians segment) {
  float height_step = height_margin / num_beams_v;
  float angle_step = segment.val() / num_beams_h;
  Cloud::Ptr cloud(new Cloud);
  for (float height = -height_margin * 0.5f; height < height_margin * 0.5f;
       height += height_step) {
    for (float angle = -segment.val() * 0.5f; angle < segment.val() * 0.5f;
         angle += angle_step) {
      RichPoint point;
      point.x() = radius * cos(angle);
      point.y() = radius * sin(angle);
      point.z() = height;
      cloud->push_back(point);
    }
  }
  return cloud;
}
