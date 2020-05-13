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
