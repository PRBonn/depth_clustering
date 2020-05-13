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
