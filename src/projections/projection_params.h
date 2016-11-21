// Copyright (C) 2016  I. Bogoslavskyi, C. Stachniss, University of Bonn

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

#include <vector>
#include <string>
#include <memory>

#include "utils/radians.h"

namespace depth_clustering {

/**
 * @brief      Class for projection parameters.
 */
class ProjectionParams {
 public:
  enum class Direction { HORIZONTAL, VERTICAL };
  enum class Set { COLS, ROWS };
  ProjectionParams() {}
  ~ProjectionParams() {}

  /**
   * @brief      Sets the span.
   *
   * @param[in]  start_angle  The start angle
   * @param[in]  end_angle    The end angle
   * @param[in]  step         The step
   * @param[in]  direction    The direction
   */
  void SetSpan(const Radians& start_angle, const Radians& end_angle,
               const Radians& step, const Direction& direction);

  /**
   * @brief      Sets the span.
   *
   * @param[in]  start_angle  The start angle
   * @param[in]  end_angle    The end angle
   * @param[in]  num_bins     The number bins
   * @param[in]  direction    The direction
   */
  void SetSpan(const Radians& start_angle, const Radians& end_angle,
               const int num_bins, const Direction& direction);

  inline const Radians& v_start_angle() const { return _v_start_angle; }
  inline const Radians& v_end_angle() const { return _v_end_angle; }
  inline const Radians& v_span() const { return _v_span; }

  inline const Radians& h_start_angle() const { return _h_start_angle; }
  inline const Radians& h_end_angle() const { return _h_start_angle; }
  inline const Radians& h_span() const { return _h_span; }
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
   * @brief      { function_description }
   *
   * @param[in]  angle  The angle
   *
   * @return     { description_of_the_return_value }
   */
  size_t RowFromAngle(const Radians& angle) const;
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
  std::vector<Radians> FillVector(const Radians& start_angle,
                                  const Radians& end_angle,
                                  const Radians& step);

  static size_t FindClosest(const std::vector<Radians>& vec,
                            const Radians& val);

  void FillCosSin();

  Radians _v_start_angle;
  Radians _v_end_angle;
  Radians _v_span;

  Radians _h_start_angle;
  Radians _h_end_angle;
  Radians _h_span;

  std::vector<Radians> _col_angles;
  std::vector<Radians> _row_angles;

  std::vector<float> _col_angles_sines;
  std::vector<float> _col_angles_cosines;

  std::vector<float> _row_angles_sines;
  std::vector<float> _row_angles_cosines;
};

}  // namespace depth_clustering

#endif  // SRC_PROJECTIONS_PROJECTION_PARAMS_H_
