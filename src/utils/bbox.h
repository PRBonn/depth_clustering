// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_UTILS_BBOX_H_
#define SRC_UTILS_BBOX_H_

#include <Eigen/Core>

#include <algorithm>
#include <limits>

#include "utils/cloud.h"

namespace dc = depth_clustering;

/**
 * @brief      Class for bounding box.
 */
class Bbox {
 public:
  static constexpr float WRONG_VOLUME = -1.0f;

  Bbox() = default;

  explicit Bbox(const depth_clustering::Cloud& cloud);

  Bbox(const Eigen::Vector3f& min_point, const Eigen::Vector3f& max_point);

  /**
   * @brief      Check if this bounding box intersects another one.
   *
   * @param[in]  other  The other bounding box.
   *
   * @return     true if intersects, false otherwise
   */
  bool Intersects(const Bbox& other) const;

  /**
   * @brief      Find the bounding box of the intersection of two boxes.
   *
   * @param[in]  other  The other bounding box.
   *
   * @return     A bounding box of the intersection.
   */
  Bbox Intersect(const Bbox& other) const;

  /**
   * @brief      Return volume of the bounding box.
   *
   * @return     Volume of the bounding box.
   */

  inline void MoveBy(const dc::Pose& pose) {
    auto moved_min = pose * _min_point;
    auto moved_max = pose * _max_point;
    _min_point = Eigen::Vector3f(std::min(moved_min.x(), moved_max.x()),
                                 std::min(moved_min.y(), moved_max.y()),
                                 std::min(moved_min.z(), moved_max.z()));
    _max_point = Eigen::Vector3f(std::max(moved_min.x(), moved_max.x()),
                                 std::max(moved_min.y(), moved_max.y()),
                                 std::max(moved_min.z(), moved_max.z()));
    UpdateScaleAndCenter();
  }

  inline float volume() const { return _volume; }
  inline Eigen::Vector3f center() const { return _center; }
  inline Eigen::Vector3f scale() const { return _scale; }
  inline const Eigen::Vector3f& min_point() const { return _min_point; }
  inline const Eigen::Vector3f& max_point() const { return _max_point; }

  inline float GetScaleX() const { return _scale.x(); }
  inline float GetScaleY() const { return _scale.y(); }
  inline float GetScaleZ() const { return _scale.z(); }

  void UpdateScaleAndCenter();

 private:
  Eigen::Vector3f _max_point =
      Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest());
  Eigen::Vector3f _min_point = Eigen::Vector3f(
      std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max());

  Eigen::Vector3f _center = Eigen::Vector3f::Zero();
  Eigen::Vector3f _scale = Eigen::Vector3f::Zero();

  float _volume = WRONG_VOLUME;
};

#endif  // SRC_UTILS_BBOX_H_
