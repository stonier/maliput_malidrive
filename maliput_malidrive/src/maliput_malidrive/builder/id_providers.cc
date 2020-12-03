// Copyright 2019 Toyota Research Institute
#include "maliput_malidrive/builder/id_providers.h"

#include <string>

namespace malidrive {
namespace builder {

maliput::api::BranchPointId GetBranchPointId(int branch_point_index) {
  return maliput::api::BranchPointId(std::to_string(branch_point_index));
}

maliput::api::JunctionId GetJunctionId(int xodr_track_id, int xodr_lane_section_index) {
  return maliput::api::JunctionId(std::to_string(xodr_track_id) + "_" + std::to_string(xodr_lane_section_index));
}

maliput::api::JunctionId GetJunctionId(int xodr_junction_id) {
  return maliput::api::JunctionId(std::to_string(xodr_junction_id));
}

maliput::api::LaneId GetLaneId(int xodr_track_id, int xodr_lane_section_index, int xodr_lane_id) {
  return maliput::api::LaneId(std::to_string(xodr_track_id) + "_" + std::to_string(xodr_lane_section_index) + "_" +
                              std::to_string(xodr_lane_id));
}

maliput::api::SegmentId GetSegmentId(int xodr_track_id, int xodr_lane_section_index) {
  return maliput::api::SegmentId(std::to_string(xodr_track_id) + "_" + std::to_string(xodr_lane_section_index));
}

maliput::api::rules::SpeedLimitRule::Id GetSpeedLimitId(const maliput::api::LaneId& lane_id, int speed_limit_index) {
  return maliput::api::rules::SpeedLimitRule::Id(lane_id.string() + "_" + std::to_string(speed_limit_index));
}

maliput::api::rules::DirectionUsageRule::Id GetDirectionUsageRuleId(const maliput::api::LaneId& lane_id,
                                                                    int direction_usage_index) {
  return maliput::api::rules::DirectionUsageRule::Id(lane_id.string() + "_" + std::to_string(direction_usage_index));
}

maliput::api::rules::DirectionUsageRule::State::Id GetDirectionUsageRuleStateId(
    const maliput::api::rules::DirectionUsageRule::Id& rule_id) {
  return maliput::api::rules::DirectionUsageRule::State::Id(rule_id.string());
}

maliput::api::rules::Rule::Id GetRuleIdFrom(const maliput::api::rules::Rule::TypeId& rule_type_id,
                                            const maliput::api::LaneId& lane_id) {
  return maliput::api::rules::Rule::Id(rule_type_id.string() + "/" + lane_id.string());
}

maliput::api::rules::Rule::Id GetRuleIdFrom(const maliput::api::rules::Rule::TypeId& rule_type_id,
                                            const maliput::api::LaneId& lane_id, int index) {
  return maliput::api::rules::Rule::Id(rule_type_id.string() + "/" + lane_id.string() + "_" + std::to_string(index));
}

}  // namespace builder
}  // namespace malidrive
