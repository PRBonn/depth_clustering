// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

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
