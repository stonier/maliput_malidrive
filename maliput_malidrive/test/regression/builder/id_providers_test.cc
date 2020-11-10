// Copyright 2019 Toyota Research Institute
#include "maliput_malidrive/builder/id_providers.h"

#include <gtest/gtest.h>

using maliput::api::BranchPointId;
using maliput::api::JunctionId;
using maliput::api::LaneId;
using maliput::api::SegmentId;
using maliput::api::rules::DirectionUsageRule;
using maliput::api::rules::Rule;
using maliput::api::rules::SpeedLimitRule;

namespace malidrive {
namespace builder {
namespace tests {

class IdProviderTest : public ::testing::Test {
 protected:
  const BranchPointId kBranchPointId{"5"};
  const int kBranchPointIndex{5};
  const DirectionUsageRule::Id kDirectionUsageRuleId{"13_4_-8_12"};
  const int kDirectionUsageRuleIndex{12};
  const DirectionUsageRule::State::Id kDirectionUsageRuleStateId{"13_4_-8_12"};
  const JunctionId kJunctionIdA{"13"};
  const JunctionId kJunctionIdB{"13_4"};
  const int kJunctionIndexA{13};
  const int kJunctionIndexB{4};
  const LaneId kLaneId{"13_4_-8"};
  const int kLaneIndex{-8};
  const Rule::Id kRuleId{"RuleTypeId/13_4_-8"};
  const Rule::TypeId kRuleTypeId{"RuleTypeId"};
  const SegmentId kSegmentId{"13_4"};
  const int kSegmentIndex{4};
  const SpeedLimitRule::Id kSpeedLimitRuleId{"13_4_-8_27"};
  const int kSpeedLimitRuleIndex{27};
};

TEST_F(IdProviderTest, BranchPointIdTest) { EXPECT_EQ(GetBranchPointId(kBranchPointIndex), kBranchPointId); }

TEST_F(IdProviderTest, JunctionIdTest) {
  EXPECT_EQ(GetJunctionId(kJunctionIndexA), kJunctionIdA);
  EXPECT_EQ(GetJunctionId(kJunctionIndexA, kJunctionIndexB), kJunctionIdB);
}

TEST_F(IdProviderTest, SegmentIdTest) { EXPECT_EQ(GetSegmentId(kJunctionIndexA, kSegmentIndex), kSegmentId); }

TEST_F(IdProviderTest, LaneIdTest) { EXPECT_EQ(GetLaneId(kJunctionIndexA, kSegmentIndex, kLaneIndex), kLaneId); }

TEST_F(IdProviderTest, SpeedLimitIdTest) {
  EXPECT_EQ(GetSpeedLimitId(kLaneId, kSpeedLimitRuleIndex), kSpeedLimitRuleId);
}

TEST_F(IdProviderTest, DirectionUsageIdTest) {
  EXPECT_EQ(GetDirectionUsageRuleId(kLaneId, kDirectionUsageRuleIndex), kDirectionUsageRuleId);
}

TEST_F(IdProviderTest, DirectionUsageStateIdTest) {
  EXPECT_EQ(GetDirectionUsageRuleStateId(kDirectionUsageRuleId), kDirectionUsageRuleStateId);
}

TEST_F(IdProviderTest, RuleIdTest) { EXPECT_EQ(GetRuleIdFrom(kRuleTypeId, kLaneId), kRuleId); }

GTEST_TEST(UniqueIntegerProviderTest, ConstructorTest) {
  const int kBaseId{123};
  const UniqueIntegerProvider dut(kBaseId);
  EXPECT_EQ(dut.get_last_id(), kBaseId);
}

GTEST_TEST(UniqueIntegerProviderTest, NewIdTest) {
  const int kBaseId{123};
  UniqueIntegerProvider dut(kBaseId);
  EXPECT_EQ(dut.new_id(), kBaseId + 1);
  EXPECT_EQ(dut.get_last_id(), kBaseId + 1);
}

}  // namespace tests
}  // namespace builder
}  // namespace malidrive
