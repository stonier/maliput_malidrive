// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lane_offset.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(LaneOffset, EqualityOperator) {
  const LaneOffset kLaneOffset{1.1 /* s */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};
  LaneOffset lane_offset = kLaneOffset;

  EXPECT_EQ(kLaneOffset, lane_offset);
  lane_offset.s_0 = 5.;
  EXPECT_NE(kLaneOffset, lane_offset);
  lane_offset.s_0 = 1.1;
  lane_offset.a = 5.;
  EXPECT_NE(kLaneOffset, lane_offset);
  lane_offset.a = 2.2;
  lane_offset.b = 10.;
  EXPECT_NE(kLaneOffset, lane_offset);
  lane_offset.b = 3.3;
  lane_offset.c = 10.;
  EXPECT_NE(kLaneOffset, lane_offset);
  lane_offset.c = 4.4;
  lane_offset.d = 10.;
  EXPECT_NE(kLaneOffset, lane_offset);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
