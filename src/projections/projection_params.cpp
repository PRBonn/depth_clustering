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

void ProjectionParams::SetSpan(const SpanParams& span_params,
                               const SpanParams::Direction& direction) {
  vector<SpanParams> params_vec = {{span_params}};
  this->SetSpan(params_vec, direction);
}

void ProjectionParams::SetSpan(const vector<SpanParams>& span_params,
                               const SpanParams::Direction& direction) {
  int num_beams = 0;
  for (const auto& span : span_params) {
    num_beams += span.num_beams();
  }
  switch (direction) {
    case SpanParams::Direction::HORIZONTAL:
      _h_span_params = SpanParams(span_params.front().start_angle(),
                                  span_params.back().end_angle(), num_beams);
      _col_angles = FillVector(span_params);
      break;
    case SpanParams::Direction::VERTICAL:
      _v_span_params = SpanParams(span_params.front().start_angle(),
                                  span_params.back().end_angle(), num_beams);
      _row_angles = FillVector(span_params);
      break;
  }
  FillCosSin();
}

vector<Radians> ProjectionParams::FillVector(const SpanParams& span_params) {
  vector<SpanParams> params_vec = {{span_params}};
  return this->FillVector(params_vec);
}

vector<Radians> ProjectionParams::FillVector(
    const vector<SpanParams>& span_params) {
  vector<Radians> res;
  for (const auto span_param : span_params) {
    Radians rad = span_param.start_angle();
    for (int i = 0; i < span_param.num_beams(); ++i) {
      res.push_back(rad);
      rad += span_param.step();
    }
  }
  return res;
}

bool ProjectionParams::valid() {
  bool all_params_valid = _v_span_params.valid() && _h_span_params.valid();
  bool arrays_empty = _row_angles.empty() && _col_angles.empty();
  bool cos_sin_empty = _row_angles_sines.empty() &&
                       _row_angles_cosines.empty() &&
                       _col_angles_sines.empty() && _col_angles_cosines.empty();
  if (!all_params_valid) {
    throw std::runtime_error("Projection parameters invalid.");
  }
  if (arrays_empty) {
    throw std::runtime_error("Projection parameters arrays not filled.");
  }
  if (cos_sin_empty) {
    throw std::runtime_error(
        "Projection parameters sin and cos arrays not filled.");
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
  params.SetSpan(SpanParams(-180_deg, 180_deg, 870),
                 SpanParams::Direction::HORIZONTAL);
  params.SetSpan(SpanParams(15_deg, -15_deg, 16),
                 SpanParams::Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

std::unique_ptr<ProjectionParams> ProjectionParams::HDL_32() {
  auto params = ProjectionParams();
  params.SetSpan(SpanParams(-180_deg, 180_deg, 870),
                 SpanParams::Direction::HORIZONTAL);
  params.SetSpan(SpanParams(10.0_deg, -30.0_deg, 32),
                 SpanParams::Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

std::unique_ptr<ProjectionParams> ProjectionParams::HDL_64_EQUAL() {
  auto params = ProjectionParams();
  params.SetSpan(SpanParams(-180_deg, 180_deg, 870),
                 SpanParams::Direction::HORIZONTAL);
  params.SetSpan(SpanParams(2.0_deg, -24.0_deg, 64),
                 SpanParams::Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

std::unique_ptr<ProjectionParams> ProjectionParams::HDL_64() {
  auto params = ProjectionParams();
  params.SetSpan(SpanParams(-180_deg, 180_deg, 870),
                 SpanParams::Direction::HORIZONTAL);
  SpanParams span_top(2.0_deg, -8.5_deg, 32);
  SpanParams span_bottom(-8.87_deg, -24.87_deg, 32);
  vector<SpanParams> spans = {{span_top, span_bottom}};
  params.SetSpan(spans, SpanParams::Direction::VERTICAL);
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
  params.SetSpan(SpanParams(-180_deg, 180_deg, discretization),
                 SpanParams::Direction::HORIZONTAL);
  params.SetSpan(SpanParams(-90_deg, 90_deg, discretization),
                 SpanParams::Direction::VERTICAL);
  params.FillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return mem_utils::make_unique<ProjectionParams>(params);
}

std::unique_ptr<ProjectionParams> ProjectionParams::FromConfigFile(
    const std::string& path) {
  fprintf(stderr, "INFO: Set en_US.UTF-8 locale.\n");
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
      params._h_span_params =
          SpanParams(Radians::FromDegrees(std::stod(str_angles[2])),
                     Radians::FromDegrees(std::stod(str_angles[3])), cols);
      fprintf(stderr, "start:%f, stop:%f, span:%f, step:%f\n",
              params._h_span_params.start_angle().ToDegrees(),
              params._h_span_params.end_angle().ToDegrees(),
              params._h_span_params.span().ToDegrees(),
              params._h_span_params.step().ToDegrees());

      // fill the cols spacing
      for (int c = 0; c < cols; ++c) {
        params._col_angles.push_back(params._h_span_params.start_angle() +
                                     params._h_span_params.step() * c);
      }

      // fill the rows
      params._v_span_params =
          SpanParams(Radians::FromDegrees(std::stod(str_angles[4])),
                     Radians::FromDegrees(std::stod(str_angles.back())), rows);
      // fill the rows with respect to img.cfg spacings
      for (size_t i = 4; i < str_angles.size(); ++i) {
        params._row_angles.push_back(
            Radians::FromDegrees(std::stof(str_angles[i])));
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
