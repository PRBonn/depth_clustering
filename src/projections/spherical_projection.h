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

#ifndef SRC_PROJECTIONS_SPHERICAL_PROJECTION_H_
#define SRC_PROJECTIONS_SPHERICAL_PROJECTION_H_

#include <opencv/cv.h>

#include <vector>

#include "projections/cloud_projection.h"
#include "projections/projection_params.h"
#include "utils/cloud.h"

namespace depth_clustering {

/**
 * @brief      Class for spherical projection.
 */
class SphericalProjection : public CloudProjection {
 public:
  explicit SphericalProjection(const ProjectionParams& projection_params)
      : CloudProjection(projection_params) {}

  /**
   * @brief      Generate projection from points
   *
   * @param[in]  points  The points
   */
  void InitFromPoints(const RichPoint::AlignedVector& points) override;
  typename CloudProjection::Ptr Clone() const override;
  virtual ~SphericalProjection() {}
};

}  // namespace depth_clustering

#endif  // SRC_PROJECTIONS_SPHERICAL_PROJECTION_H_
