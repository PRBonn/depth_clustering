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

#ifndef SRC_IMAGE_LABELERS_ABSTRACT_IMAGE_LABELER_H_
#define SRC_IMAGE_LABELERS_ABSTRACT_IMAGE_LABELER_H_

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "image_labelers/diff_helpers/diff_factory.h"
#include "projections/projection_params.h"
#include "utils/radians.h"

namespace depth_clustering {

/**
 * @brief      Class for abstract image labeler.
 * @details    This class is responsible for labeling an given depth image based
 *             on the provided angle_threshold.
 */
class AbstractImageLabeler {
 public:
  explicit AbstractImageLabeler(const cv::Mat& depth_image,
                                const ProjectionParams& params,
                                const Radians& angle_threshold)
      : _depth_image_ptr(&depth_image),
        _params(params),
        _radians_threshold(angle_threshold.val()) {
    _label_image =
        cv::Mat::zeros(_depth_image_ptr->size(), cv::DataType<uint16_t>::type);
  }
  virtual ~AbstractImageLabeler() {}

  void SetDepthImage(const cv::Mat& depth_image) {
    _depth_image_ptr = &depth_image;
  }

  /**
   * @brief      An interface for children to compute labels
   */
  virtual void ComputeLabels(DiffFactory::DiffType diff_type) = 0;

  /**
   * @brief      Gets the label image.
   *
   * @return     The label image.
   */
  inline const cv::Mat* GetLabelImage() const { return &_label_image; }

  /**
   * @brief      Generates random-colored image from image of labels
   *
   * @param[in]  label_image  The label image
   *
   * @return     Random-colored image from label image
   */
  static cv::Mat LabelsToColor(const cv::Mat& label_image);

 protected:
  const cv::Mat* _depth_image_ptr;
  ProjectionParams _params;
  cv::Mat _label_image;
  float _radians_threshold;

  static constexpr std::array<std::array<int, 3>, 200> RANDOM_COLORS = {{
      {{104, 109, 253}}, {{125, 232, 153}}, {{158, 221, 134}},
      {{228, 109, 215}}, {{249, 135, 210}}, {{255, 207, 237}},
      {{151, 120, 235}}, {{145, 123, 213}}, {{172, 243, 184}},
      {{105, 131, 110}}, {{217, 253, 154}}, {{250, 102, 109}},
      {{116, 179, 127}}, {{200, 251, 206}}, {{117, 146, 240}},
      {{234, 162, 176}}, {{160, 172, 171}}, {{205, 129, 168}},
      {{197, 167, 238}}, {{234, 248, 101}}, {{226, 240, 119}},
      {{189, 211, 231}}, {{226, 170, 216}}, {{109, 180, 162}},
      {{115, 167, 221}}, {{162, 134, 131}}, {{203, 169, 114}},
      {{221, 138, 114}}, {{246, 146, 237}}, {{200, 167, 244}},
      {{198, 150, 236}}, {{237, 235, 191}}, {{132, 137, 171}},
      {{136, 219, 103}}, {{229, 210, 135}}, {{133, 188, 111}},
      {{142, 144, 142}}, {{122, 189, 120}}, {{127, 142, 229}},
      {{249, 147, 235}}, {{255, 195, 148}}, {{202, 126, 227}},
      {{135, 195, 159}}, {{139, 173, 142}}, {{123, 118, 246}},
      {{254, 186, 204}}, {{184, 138, 221}}, {{112, 160, 229}},
      {{243, 165, 249}}, {{200, 194, 254}}, {{172, 205, 151}},
      {{196, 132, 119}}, {{240, 251, 116}}, {{186, 189, 147}},
      {{154, 162, 144}}, {{178, 103, 147}}, {{139, 188, 175}},
      {{156, 163, 178}}, {{225, 244, 174}}, {{118, 227, 101}},
      {{176, 178, 120}}, {{113, 105, 164}}, {{137, 105, 123}},
      {{144, 114, 196}}, {{163, 115, 216}}, {{143, 128, 133}},
      {{221, 225, 169}}, {{165, 152, 214}}, {{133, 163, 101}},
      {{212, 202, 171}}, {{134, 255, 128}}, {{217, 201, 143}},
      {{213, 175, 151}}, {{149, 234, 191}}, {{242, 127, 242}},
      {{152, 189, 230}}, {{152, 121, 249}}, {{234, 253, 138}},
      {{152, 234, 147}}, {{171, 195, 244}}, {{254, 178, 194}},
      {{205, 105, 153}}, {{226, 234, 202}}, {{153, 136, 236}},
      {{248, 242, 137}}, {{162, 251, 207}}, {{152, 126, 144}},
      {{180, 213, 122}}, {{230, 185, 113}}, {{118, 148, 223}},
      {{162, 124, 183}}, {{180, 247, 119}}, {{120, 223, 121}},
      {{252, 124, 181}}, {{254, 174, 165}}, {{188, 186, 210}},
      {{254, 137, 161}}, {{216, 222, 120}}, {{215, 247, 128}},
      {{121, 240, 179}}, {{135, 122, 215}}, {{255, 131, 237}},
      {{224, 112, 171}}, {{167, 223, 219}}, {{103, 200, 161}},
      {{112, 154, 156}}, {{170, 127, 228}}, {{133, 145, 244}},
      {{244, 100, 101}}, {{254, 199, 148}}, {{120, 165, 205}},
      {{112, 121, 141}}, {{175, 135, 134}}, {{221, 250, 137}},
      {{247, 245, 231}}, {{236, 109, 115}}, {{169, 198, 194}},
      {{196, 195, 136}}, {{138, 255, 145}}, {{239, 141, 147}},
      {{194, 220, 253}}, {{149, 209, 204}}, {{241, 127, 132}},
      {{226, 184, 108}}, {{222, 108, 147}}, {{109, 166, 185}},
      {{152, 107, 167}}, {{153, 117, 222}}, {{165, 171, 214}},
      {{189, 196, 243}}, {{248, 235, 129}}, {{120, 198, 202}},
      {{223, 206, 134}}, {{175, 114, 214}}, {{115, 196, 189}},
      {{157, 141, 112}}, {{111, 161, 201}}, {{207, 183, 214}},
      {{201, 164, 235}}, {{168, 187, 154}}, {{114, 176, 229}},
      {{151, 163, 221}}, {{134, 160, 173}}, {{103, 112, 168}},
      {{209, 169, 218}}, {{137, 220, 119}}, {{168, 220, 210}},
      {{182, 192, 194}}, {{233, 187, 120}}, {{223, 185, 160}},
      {{120, 232, 147}}, {{165, 169, 124}}, {{251, 159, 129}},
      {{182, 114, 178}}, {{159, 116, 158}}, {{217, 121, 122}},
      {{106, 229, 235}}, {{164, 208, 214}}, {{180, 178, 142}},
      {{110, 206, 136}}, {{238, 152, 205}}, {{109, 245, 253}},
      {{213, 232, 131}}, {{215, 134, 100}}, {{163, 140, 135}},
      {{233, 198, 143}}, {{221, 129, 224}}, {{150, 179, 137}},
      {{171, 128, 119}}, {{210, 245, 246}}, {{209, 111, 161}},
      {{237, 133, 194}}, {{166, 157, 255}}, {{191, 206, 225}},
      {{125, 135, 110}}, {{199, 188, 196}}, {{196, 101, 202}},
      {{237, 211, 167}}, {{134, 118, 177}}, {{110, 179, 126}},
      {{196, 182, 196}}, {{150, 211, 218}}, {{162, 118, 228}},
      {{150, 209, 185}}, {{219, 151, 148}}, {{201, 168, 104}},
      {{237, 146, 123}}, {{234, 163, 146}}, {{213, 251, 127}},
      {{227, 152, 214}}, {{230, 195, 100}}, {{136, 117, 222}},
      {{180, 132, 173}}, {{112, 226, 113}}, {{198, 155, 126}},
      {{149, 255, 152}}, {{223, 124, 170}}, {{104, 146, 255}},
      {{113, 205, 183}}, {{100, 156, 216}},
  }};
};

}  // namespace depth_clustering

#endif  // SRC_IMAGE_LABELERS_ABSTRACT_IMAGE_LABELER_H_
