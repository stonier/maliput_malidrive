// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/elevation_profile.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(Elevation, EqualityOperator) {
  const ElevationProfile::Elevation kElevation{1.1 /* s0 */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};
  ElevationProfile::Elevation elevation = kElevation;

  EXPECT_EQ(kElevation, elevation);
  elevation.s_0 = 5.;
  EXPECT_NE(kElevation, elevation);
  elevation.s_0 = 1.1;
  elevation.a = 5.;
  EXPECT_NE(kElevation, elevation);
  elevation.a = 2.2;
  elevation.b = 10.;
  EXPECT_NE(kElevation, elevation);
  elevation.b = 3.3;
  elevation.c = 10.;
  EXPECT_NE(kElevation, elevation);
  elevation.c = 4.4;
  elevation.d = 10.;
  EXPECT_NE(kElevation, elevation);
}

GTEST_TEST(ElevationProfile, EqualityOperator) {
  const ElevationProfile kElevationProfile{{{1.23 /* s_0 */, 523.2 /* a */, 83.27 /* b */, 0.77 /* c */, 100. /* d */},
                                            {1.215 /* s_0 */, 1.2 /* a */, 35.27 /* b */, 0.2 /* c */, 564. /* d */}}};
  ElevationProfile elevation_profile = kElevationProfile;

  EXPECT_EQ(kElevationProfile, elevation_profile);
  elevation_profile.elevations[0].c = 568.5;
  EXPECT_NE(kElevationProfile, elevation_profile);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
