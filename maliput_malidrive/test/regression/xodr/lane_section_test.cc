// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lane_section.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(LaneSection, EqualityOperator) {
  const LaneSection kLaneSection{
      2.1 /* s_0 */,
      true /* single_side */,
      {{Lane::Id("test_id1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{1.1, 2.2}}}} /* left_lanes */,
      {Lane::Id("test_id2"), Lane::Type::kShoulder, false, {}, {}} /* center_lane */,
      {{Lane::Id("test_id3"), Lane::Type::kRestricted, false, {}, std::vector<LaneWidth>{{5.5, 6.6}}}} /* right_lanes */
  };
  LaneSection lane_section = kLaneSection;
  EXPECT_EQ(kLaneSection, lane_section);
  lane_section.s_0 = 993.;
  EXPECT_NE(kLaneSection, lane_section);
  lane_section.s_0 = 2.1;
  lane_section.single_side = false;
  EXPECT_NE(kLaneSection, lane_section);
  lane_section.single_side = true;
  lane_section.left_lanes[0].type = Lane::Type::kEntry;
  EXPECT_NE(kLaneSection, lane_section);
  lane_section.left_lanes[0].type = Lane::Type::kDriving;
  lane_section.center_lane.type = Lane::Type::kSidewalk;
  EXPECT_NE(kLaneSection, lane_section);
  lane_section.center_lane.type = Lane::Type::kShoulder;
  lane_section.right_lanes[0].type = Lane::Type::kRoadWorks;
  EXPECT_NE(kLaneSection, lane_section);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
