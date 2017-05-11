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

#ifndef SRC_PROJECTIONS_PROJECTION_PARAMS_H_
#define SRC_PROJECTIONS_PROJECTION_PARAMS_H_

#include <memory>
#include <string>
#include <vector>

#include "utils/radians.h"
#include "utils/useful_typedefs.h"

namespace depth_clustering {

class SpanParams {
 public:
  /**
   * Enum for the direction of the span.
   */
  enum class Direction { HORIZONTAL, VERTICAL };

  SpanParams() {}
  SpanParams(const Radians& start_angle, const Radians& end_angle,
             const Radians& step) {
    _start_angle = start_angle;
    _end_angle = end_angle;
    _step = step;
    _num_beams = floor((_end_angle - _start_angle) / _step);
    _span = Radians::Abs(end_angle - start_angle);
  }

  SpanParams(const Radians& start_angle, const Radians& end_angle,
             int num_beams) {
    _start_angle = start_angle;
    _end_angle = end_angle;
    _num_beams = num_beams;
    _step = (_end_angle - _start_angle) / _num_beams;
    _span = Radians::Abs(end_angle - start_angle);
  }

  const Radians& start_angle() const { return _start_angle; }
  const Radians& end_angle() const { return _end_angle; }
  const Radians& step() const { return _step; }
  const Radians& span() const { return _span; }
  int num_beams() const { return _num_beams; }

  bool valid() const { return _num_beams > 0 && _span > 0_deg; }

 private:
  Radians _start_angle = 0_deg;
  Radians _end_angle = 0_deg;
  Radians _step = 0_deg;
  Radians _span = 0_deg;
  int _num_beams = 0;
};

/**
 * @brief      Class for projection parameters.
 */
class ProjectionParams {
 public:
  using Ptr = shared_ptr<ProjectionParams>;
  using ConstPtr = const shared_ptr<const ProjectionParams>;

  enum class Set { COLS, ROWS };
  ProjectionParams() {}
  ~ProjectionParams() {}

  /**
   * @brief      Set the angle span in a given direction.
   *
   * @param[in]  span_params  The span parameters packad into ::SpanParams.
   * @param[in]  direction    The direction. Must be one of
   *                          SpanParams::Direction.
   */
  void SetSpan(const SpanParams& span_params,
               const SpanParams::Direction& direction);

  /**
   * @brief      Set the angle spans in a given direction.
   *
   * @param[in]  span_params  The span parameters packad into ::SpanParams
   * @param[in]  direction    The direction. Must be one of
   *                          SpanParams::Direction.
   */
  void SetSpan(const std::vector<SpanParams>& span_params,
               const SpanParams::Direction& direction);

  inline const Radians& v_start_angle() const {
    return _v_span_params.start_angle();
  }
  inline const Radians& v_end_angle() const {
    return _v_span_params.end_angle();
  }
  inline const Radians& v_span() const { return _v_span_params.span(); }

  inline const Radians& h_start_angle() const {
    return _h_span_params.start_angle();
  }
  inline const Radians& h_end_angle() const {
    return _h_span_params.end_angle();
  }
  inline const Radians& h_span() const { return _h_span_params.span(); }
  inline size_t rows() const { return _row_angles.size(); }
  inline size_t cols() const { return _col_angles.size(); }
  inline size_t size() const { return rows() * cols(); }

  /**
   * @brief      Get angle from row
   *
   * @param[in]  row   The row
   *
   * @return     Angle in radians
   */
  const Radians AngleFromRow(int row) const;

  /**
   * @brief      Get angle from col
   *
   * @param[in]  col   The col
   *
   * @return     Angle in radians
   */
  const Radians AngleFromCol(int col) const;

  /**
   * @brief      Get row number from angle
   *
   * @param[in]  angle  The angle
   *
   * @return     Row number
   */
  size_t RowFromAngle(const Radians& angle) const;

  /**
   * @brief      Get col number from angle
   *
   * @param[in]  angle  The angle
   *
   * @return     Col number
   */
  size_t ColFromAngle(const Radians& angle) const;

  const std::vector<float>& RowAngleCosines() const;
  const std::vector<float>& ColAngleCosines() const;
  const std::vector<float>& RowAngleSines() const;
  const std::vector<float>& ColAngleSines() const;

  bool valid();

  /**
   * @brief      Default parameters for 16 beam Velodyne
   *
   * @return     A pointer to parameters
   */
  static std::unique_ptr<ProjectionParams> VLP_16();
  /**
   * @brief      Default parameters for 32 beam Velodyne
   *
   * @return     A pointer to parameters
   */
  static std::unique_ptr<ProjectionParams> HDL_32();
  /**
   * @brief      Default parameters for 64 beam Velodyne
   *
   * @return     A pointer to parameters
   */
  static std::unique_ptr<ProjectionParams> HDL_64();
  /**
   * @brief      Parameters for 64 beam velodyne assuming equal spacing between
   *             the lasers.
   *
   * @return     A pointer to parameters
   */
  static std::unique_ptr<ProjectionParams> HDL_64_EQUAL();
  /**
   * @brief      Default parameters for Velodyne from config file
   *
   * @return     A pointer to parameters
   */
  static std::unique_ptr<ProjectionParams> FromConfigFile(
      const std::string& path);
  /**
   * @brief      Default parameters to cover full sphere
   *
   * @return     A pointer to parameters
   */
  static std::unique_ptr<ProjectionParams> FullSphere(
      const Radians& discretization = 5_deg);

 private:
  std::vector<Radians> FillVector(const SpanParams& span_params);
  std::vector<Radians> FillVector(const std::vector<SpanParams>& span_params);

  static size_t FindClosest(const std::vector<Radians>& vec,
                            const Radians& val);

  void FillCosSin();

  SpanParams _v_span_params;
  SpanParams _h_span_params;

  std::vector<Radians> _col_angles;
  std::vector<Radians> _row_angles;

  std::vector<float> _col_angles_sines;
  std::vector<float> _col_angles_cosines;

  std::vector<float> _row_angles_sines;
  std::vector<float> _row_angles_cosines;
};

}  // namespace depth_clustering

#endif  // SRC_PROJECTIONS_PROJECTION_PARAMS_H_
