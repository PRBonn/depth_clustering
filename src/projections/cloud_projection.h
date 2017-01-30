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

#ifndef SRC_PROJECTIONS_CLOUD_PROJECTION_H_
#define SRC_PROJECTIONS_CLOUD_PROJECTION_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <Eigen/Core>

#include <list>
#include <memory>
#include <stdexcept>
#include <vector>

#include "projections/projection_params.h"

#include "utils/radians.h"
#include "utils/rich_point.h"
#include "utils/useful_typedefs.h"

namespace depth_clustering {

/**
 * @brief      Abstract class for cloud projection.
 */
class CloudProjection {
  class PointContainer;
  // some useful usings
  using PointColumn = std::vector<PointContainer>;
  using PointMatrix = std::vector<PointColumn>;

 public:
  using Ptr = shared_ptr<CloudProjection>;
  using ConstPtr = shared_ptr<const CloudProjection>;

  enum class Type { SPHERICAL, CYLLINDRICAL };

  explicit CloudProjection(const ProjectionParams& params);
  virtual ~CloudProjection() {}

  /**
   * @brief      Initialize from 3d points.
   *
   * @param[in]  points  The points
   */
  virtual void InitFromPoints(const std::vector<RichPoint>& points) = 0;

  /**
   * @brief      Polymorphic clone of a projection.
   *
   * @return     Shared pointer of a copy of this object.
   */
  virtual CloudProjection::Ptr Clone() const = 0;

  const cv::Mat& depth_image() const;
  cv::Mat& depth_image();
  inline void CloneDepthImage(const cv::Mat& image) {
    _depth_image = image.clone();
  }

  inline size_t rows() const { return _params.rows(); }
  inline size_t cols() const { return _params.cols(); }
  inline size_t size() const { return _params.size(); }
  inline const ProjectionParams& params() const { return _params; }
  inline const PointContainer& at(const size_t row, const size_t col) const {
    return _data[col][row];
  }
  inline PointContainer& at(const size_t row, const size_t col) {
    return _data[col][row];
  }
  inline const PointMatrix& matrix() const { return _data; }

  /**
   * @brief      Check if where we store data is valid.
   *
   * @param[in]  image  The image to check
   */
  void CheckImageAndStorage(const cv::Mat& image);

  /**
   * @brief      Check if where we store data is valid.
   *
   * @param[in]  points  The points to check
   */
  void CheckCloudAndStorage(const std::vector<RichPoint>& points);

  /**
   * @brief      Unproject a point from depth image coordinate
   *
   * @param[in]  image  A depth image
   * @param[in]  row    A row in the image
   * @param[in]  col    A col in the image
   *
   * @return     { description_of_the_return_value }
   */
  virtual RichPoint UnprojectPoint(const cv::Mat& image, const int row,
                                   const int col) const;

  /**
  * @brief      Set corrections for systematic error in a dataset (see
  *             notebooks in the repo)
  *
  * @param[in]  corrections  A vector of correction in depth for every beam.
  */
  inline void SetCorrections(const std::vector<float>& corrections) {
    _corrections = corrections;
  }

  /**
   * @brief      Fix systematic error. See notebooks in the repo for details.
   */
  void FixDepthSystematicErrorIfNeeded();

 protected:
  // just stores addresses of the points. Does not own them.
  PointMatrix _data;

  ProjectionParams _params;

  cv::Mat _depth_image;

  std::vector<float> _corrections;
};

/**
 * @brief      Class for point container.
 */
class CloudProjection::PointContainer {
 public:
  PointContainer();
  inline bool IsEmpty() const { return _points.empty(); }
  inline std::list<size_t>& points() { return _points; }
  inline const std::list<size_t>& points() const { return _points; }

 private:
  std::list<size_t> _points;
};

}  // namespace depth_clustering

#endif  // SRC_PROJECTIONS_CLOUD_PROJECTION_H_
