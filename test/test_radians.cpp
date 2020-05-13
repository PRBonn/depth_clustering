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

#include <gtest/gtest.h>
#include "utils/radians.h"

using namespace depth_clustering;

TEST(RadiansTest, test_rad) {
  Radians pi = 3.14_rad;
  double eps = 0.000001;
  EXPECT_NEAR(3.14, pi.val(), eps);
}

TEST(RadiansTest, test_deg) {
  Radians pi = 180_deg;
  double eps = 0.000001;
  EXPECT_NEAR(M_PI, pi.val(), eps);
}

TEST(RadiansTest, test_copy) {
  Radians pi = 180_deg;
  Radians pi_copy(pi);
  double eps = 0.000001;
  EXPECT_NEAR(M_PI, pi.val(), eps);
  EXPECT_NEAR(M_PI, pi_copy.val(), eps);
}

TEST(RadiansTest, test_from_radians) {
  Radians pi = 180_deg;
  Radians pi_copy = Radians::FromRadians(pi.val());
  double eps = 0.000001;
  EXPECT_NEAR(M_PI, pi.val(), eps);
  EXPECT_NEAR(M_PI, pi_copy.val(), eps);
}

TEST(RadiansTest, test_plus) {
  Radians a = 2_deg;
  Radians b = 2_deg;
  Radians res = 4_deg;
  double eps = 0.000001;
  EXPECT_NEAR(res.ToDegrees(), (a + b).ToDegrees(), eps);
}

TEST(RadiansTest, test_plus_equals) {
  Radians a = -2_deg;
  Radians b = 2_deg;
  a += b;
  Radians res = 0_deg;
  double eps = 0.000001;
  EXPECT_NEAR(res.ToDegrees(), a.ToDegrees(), eps);
}
