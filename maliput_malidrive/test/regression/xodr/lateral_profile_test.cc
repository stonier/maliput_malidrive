// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lateral_profile.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(Superelevation, EqualityOperator) {
  const LateralProfile::Superelevation kSuperelevation{1.1 /* s0 */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */,
                                                       5.5 /* d */};
  LateralProfile::Superelevation superelevation = kSuperelevation;

  EXPECT_EQ(kSuperelevation, superelevation);
  superelevation.s_0 = 5.;
  EXPECT_NE(kSuperelevation, superelevation);
  superelevation.s_0 = 1.1;
  superelevation.a = 5.;
  EXPECT_NE(kSuperelevation, superelevation);
  superelevation.a = 2.2;
  superelevation.b = 10.;
  EXPECT_NE(kSuperelevation, superelevation);
  superelevation.b = 3.3;
  superelevation.c = 10.;
  EXPECT_NE(kSuperelevation, superelevation);
  superelevation.c = 4.4;
  superelevation.d = 10.;
  EXPECT_NE(kSuperelevation, superelevation);
}

GTEST_TEST(LateralProfile, EqualityOperator) {
  const LateralProfile kLateralProfile{{{1.23 /* s_0 */, 523.2 /* a */, 83.27 /* b */, 0.77 /* c */, 100. /* d */},
                                        {1.215 /* s_0 */, 1.2 /* a */, 35.27 /* b */, 0.2 /* c */, 564. /* d */}}};
  LateralProfile lateral_profile = kLateralProfile;

  EXPECT_EQ(kLateralProfile, lateral_profile);
  lateral_profile.superelevations[0].c = 568.5;
  EXPECT_NE(kLateralProfile, lateral_profile);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
