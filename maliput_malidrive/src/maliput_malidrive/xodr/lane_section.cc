// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lane_section.h"

namespace malidrive {
namespace xodr {

bool LaneSection::operator==(const LaneSection& other) const {
  return s_0 == other.s_0 && single_side == other.single_side && left_lanes == other.left_lanes &&
         center_lane == other.center_lane && right_lanes == other.right_lanes;
}

bool LaneSection::operator!=(const LaneSection& other) const { return !(*this == other); }

}  // namespace xodr
}  // namespace malidrive
