// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_TEST_HELPER_FUNCS_H_
#define SRC_TEST_HELPER_FUNCS_H_

#include "utils/cloud.h"
#include "utils/radians.h"
#include "utils/useful_typedefs.h"

namespace ds = depth_clustering;

ds::Cloud::Ptr CreateWallLikeCloud(double max_val, double step,
                                   double mult = 1.0);

ds::Cloud::Ptr CreateCyllinderCloud(float radius, int num_beams_h,
                                    int num_beams_v, float height_margin,
                                    ds::Radians segment);

ds::Cloud::Ptr CreateMovedCloud(double max_val, double step, double mult,
                                double dist);

ds::Cloud::Ptr MoveAlongY(ds::Cloud::Ptr cloud, double dist);

ds::Cloud::Ptr RotateAroundZ(ds::Cloud::Ptr cloud, ds::Radians angle);

ds::Cloud::Ptr CreateWallLikeCloudLabeled(double max_val, double step,
                                          uint32_t max_label,
                                          double mult = 1.0);

#endif  // SRC_TEST_HELPER_FUNCS_H_
