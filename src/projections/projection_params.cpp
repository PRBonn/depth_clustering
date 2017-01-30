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

#include "projections/projection_params.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include "utils/mem_utils.h"

namespace depth_clustering {

using std::vector;
using std::string;
using std::upper_bound;
using boost::algorithm::starts_with;

void ProjectionParams::SetSpan(const Radians& start_angle,
                               const Radians& end_angle, const Radians& step,
                               const ProjectionParams::Direction& direction) {
  switch (direction) {
    case Direction::HORIZONTAL:
      _h_start_angle = start_angle;
      _h_end_angle = end_angle;
      _h_span = Radians::Abs(_h_end_angle - _h_start_angle);
      _col_angles = FillVector(start_angle, end_angle, step);
      break;
    case Direction::VERTICAL:
      _v_start_angle = start_angle;
      _v_end_angle = end_angle;
      _v_span = Radians::Abs(_v_end_angle - _v_start_angle);
      _row_angles = FillVector(start_angle, end_angle, step);
      break;
  }
  FillCosSin();
}

vector<Radians> ProjectionParams::FillVector(const Radians& start_angle,
                                             const Radians& end_angle,
                                             const Radians& step) {
  vector<Radians> res;
  int num = floor((end_angle - start_angle) / step);
  res.reserve(num);

  Radians rad = start_angle;
  for (int i = 0; i < num; ++i) {
    res.push_back(rad);
    rad += step;
  }
  return res;
}

void ProjectionParams::SetSpan(const Radians& start_angle,
                               const Radians& end_angle, const int num_bins,
                               const ProjectionParams::Direction& direction) {
  Radians step = (end_angle - start_angle) / num_bins;
  this->SetSpan(start_angle, end_angle, step, direction);
}

bool ProjectionParams::valid() {
  bool all_params_valid = _v_span.valid() && _v_start_angle.valid() &&
                          _v_end_angle.valid() && _h_span.valid() &&
                          _h_start_angle.valid() && _h_end_angle.valid();
  bool arrays_empty = _row_angles.empty() && _col_angles.empty();
  bool cos_sin_empty = _row_angles_sines.empty() &&
                       _row_angles_cosines.empty() &&
                       _col_angles_sines.empty() && _col_angles_cosines.empty();
  if (!all_params_valid) {
    fprintf(stderr, "ERROR: params are invalid\n");
    return false;
  }
  if (arrays_empty) {
    fprintf(stderr, "ERROR: arrays are empty\n");
    return false;
  }
  if (cos_sin_empty) {
    fprintf(stderr, "ERROR: cosine or sine array are empty\n");
    return false;
  }
  return true;
}

const Radians ProjectionParams::AngleFromRow(int row) const {
  if (row >= 0 && static_cast<size_t>(row) < _row_angles.size()) {
    return _row_angles[row];
  }
  fprintf(stderr, "ERROR: row %d is wrong\n", row);
  return 0.0_deg;
}

const Radians ProjectionParams::AngleFromCol(int col) const {
  int actual_col = col;
  if (col < 0) {
    actual_col = col + _col_angles.size();
  } else if (static_cast<size_t>(col) >= _col_angles.size()) {
    actual_col = col - _col_angles.size();
  }
  // everything is normal
  return _col_angles[actual_col];
}

size_t ProjectionParams::RowFromAngle(const Radians& angle) const {
  return FindClosest(_row_angles, angle);
}

size_t ProjectionParams::ColFromAngle(const Radians& angle) const {
  return FindClosest(_col_angles, angle);
}

size_t ProjectionParams::FindClosest(const vector<Radians>& vec,
                                     const Radians& val) {
  size_t found = 0;
  if (vec.front() < vec.back()) {
    found = upper_bound(vec.begin(), vec.end(), val) - vec.begin();
  } else {
    found = vec.rend() - upper_bound(vec.rbegin(), vec.rend(), val);
  }
  if (found == 0) {
    return found;
  }
  if (found == vec.size()) {
    return found - 1;
  }
  auto diff_next = Radians::Abs(vec[found] - val);
  auto diff_prev = Radians::Abs(val - vec[found - 1]);
  return diff_next < diff_prev ? found : found - 1;
}

std::unique_ptr<ProjectionParams> ProjectionParams::VLP_16() {
  auto params = ProjectionParams();
  params.SetSpan(-180_deg, 180_deg, 870, Direction::HORIZONTAL);
  params.SetSpan(15_deg, -15_deg, 16, Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

std::unique_ptr<ProjectionParams> ProjectionParams::HDL_32() {
  auto params = ProjectionParams();
  params.SetSpan(-180_deg, 180_deg, 870, Direction::HORIZONTAL);
  params.SetSpan(10.0_deg, -30.0_deg, 32, Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

std::unique_ptr<ProjectionParams> ProjectionParams::HDL_64() {
  auto params = ProjectionParams();
  params.SetSpan(-180_deg, 180_deg, 870, Direction::HORIZONTAL);
  params.SetSpan(2.0_deg, -24.0_deg, 64, Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

std::unique_ptr<ProjectionParams> ProjectionParams::FullSphere(
    const Radians& discretization) {
  auto params = ProjectionParams();
  params.SetSpan(-180_deg, 180_deg, discretization, Direction::HORIZONTAL);
  params.SetSpan(-90_deg, 90_deg, discretization, Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

std::unique_ptr<ProjectionParams> ProjectionParams::FromConfigFile(
    const std::string& path) {
  std::locale::global(std::locale("en_US.UTF-8"));
  fprintf(stderr, "INFO: Reading config.\n");
  ProjectionParams params;
  // we need to fill this thing. Parsing text files again. Is that what PhD in
  // Robotics is about?
  std::ifstream file(path.c_str());
  if (!file.is_open()) {
    fprintf(stderr, "ERROR: cannot open file: '%s'\n", path.c_str());
    return nullptr;
  }
  for (std::string line; std::getline(file, line, '\n');) {
    if (starts_with(line, "#")) {
      // we have found a commentary
      fprintf(stderr, "INFO: Skipping commentary: \n\t %s\n", line.c_str());
    } else {
      // here we parse the line
      vector<string> str_angles;
      boost::split(str_angles, line, boost::is_any_of(";"));
      if (str_angles.size() < 4) {
        fprintf(stderr, "ERROR: format of line is wrong.\n");
        return nullptr;
      }
      int cols = std::stoi(str_angles[0]);
      int rows = std::stoi(str_angles[1]);
      params._h_start_angle = Radians::FromDegrees(std::stod(str_angles[2]));
      params._h_end_angle = Radians::FromDegrees(std::stod(str_angles[3]));
      params._h_span =
          Radians::Abs(params._h_end_angle - params._h_start_angle);
      auto step = (params._h_end_angle - params._h_start_angle) / cols;
      fprintf(stderr, "start:%f, stop:%f, span:%f, step:%f\n",
              params._h_start_angle.ToDegrees(),
              params._h_end_angle.ToDegrees(), params._h_span.ToDegrees(),
              step.ToDegrees());

      // fill the cols spacing
      for (int c = 0; c < cols; ++c) {
        params._col_angles.push_back(params._h_start_angle + step * c);
      }

      // fill the rows
      params._v_start_angle = Radians::FromDegrees(std::stod(str_angles[4]));
      params._v_end_angle = Radians::FromDegrees(std::stod(str_angles.back()));
      params._v_span =
          Radians::Abs(params._v_end_angle - params._v_start_angle);
      // fill the rows with respect to img.cfg spacings
      for (size_t i = 4; i < str_angles.size(); ++i) {
        params._row_angles.push_back(
            Radians::FromDegrees(std::stod(str_angles[i])));
      }
      if (params._row_angles.size() != static_cast<size_t>(rows)) {
        fprintf(stderr, "ERROR: wrong config\n");
        return nullptr;
      }
    }
  }
  // fill cos and sin arrays
  params.FillCosSin();
  // check validity
  if (!params.valid()) {
    fprintf(stderr, "ERROR: the config read was not valid.\n");
    return nullptr;
  }
  fprintf(stderr, "INFO: Params sucessfully read. Rows: %lu, Cols: %lu\n",
          params._row_angles.size(), params._col_angles.size());
  return mem_utils::make_unique<ProjectionParams>(params);
}

const std::vector<float>& ProjectionParams::RowAngleCosines() const {
  return _row_angles_cosines;
}
const std::vector<float>& ProjectionParams::ColAngleCosines() const {
  return _col_angles_cosines;
}
const std::vector<float>& ProjectionParams::RowAngleSines() const {
  return _row_angles_sines;
}
const std::vector<float>& ProjectionParams::ColAngleSines() const {
  return _col_angles_sines;
}

void ProjectionParams::FillCosSin() {
  _row_angles_sines.clear();
  _row_angles_cosines.clear();
  for (const auto& angle : _row_angles) {
    _row_angles_sines.push_back(sin(angle.val()));
    _row_angles_cosines.push_back(cos(angle.val()));
  }
  _col_angles_sines.clear();
  _col_angles_cosines.clear();
  for (const auto& angle : _col_angles) {
    _col_angles_sines.push_back(sin(angle.val()));
    _col_angles_cosines.push_back(cos(angle.val()));
  }
}

}  // namespace depth_clustering
