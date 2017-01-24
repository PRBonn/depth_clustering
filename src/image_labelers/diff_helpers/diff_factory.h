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

#ifndef SRC_IMAGE_LABELERS_DIFF_HELPERS_DIFF_FACTORY_H_
#define SRC_IMAGE_LABELERS_DIFF_HELPERS_DIFF_FACTORY_H_

#include <memory>

#include "image_labelers/diff_helpers/abstract_diff.h"
#include "image_labelers/diff_helpers/angle_diff.h"
#include "image_labelers/diff_helpers/line_dist_diff.h"
#include "image_labelers/diff_helpers/simple_diff.h"
#include "utils/mem_utils.h"

namespace depth_clustering {

class DiffFactory {
 public:
  enum class DiffType {
    SIMPLE,
    ANGLES,
    ANGLES_PRECOMPUTED,
    LINE_DIST,
    LINE_DIST_PRECOMPUTED,
    NONE
  };

  static std::unique_ptr<AbstractDiff> Build(
      DiffType type, const cv::Mat* source_image,
      const ProjectionParams* params = nullptr) {
    switch (type) {
      case DiffType::SIMPLE: {
        return std::unique_ptr<AbstractDiff>(new SimpleDiff(source_image));
        break;
      }
      case DiffType::ANGLES: {
        return std::unique_ptr<AbstractDiff>(
            new AngleDiff(source_image, params));
        break;
      }
      case DiffType::ANGLES_PRECOMPUTED: {
        return std::unique_ptr<AbstractDiff>(
            new AngleDiffPrecomputed(source_image, params));
        break;
      }
      case DiffType::LINE_DIST: {
        return std::unique_ptr<AbstractDiff>(
            new LineDistDiff(source_image, params));
        break;
      }
      case DiffType::LINE_DIST_PRECOMPUTED: {
        return std::unique_ptr<AbstractDiff>(
            new LineDistDiffPrecomputed(source_image, params));
        break;
      }
      case DiffType::NONE: {
        fprintf(stderr, "ERROR: DiffType is NONE. Please set it.\n");
        break;
      }
    }
    return nullptr;
  }
};

}  // namespace depth_clustering
#endif  // SRC_IMAGE_LABELERS_DIFF_HELPERS_DIFF_FACTORY_H_
