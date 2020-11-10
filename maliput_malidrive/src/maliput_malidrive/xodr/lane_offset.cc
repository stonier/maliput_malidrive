// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lane_offset.h"

namespace malidrive {
namespace xodr {

bool LaneOffset::operator==(const LaneOffset& other) const {
  return s_0 == other.s_0 && a == other.a && b == other.b && c == other.c && d == other.d;
}

bool LaneOffset::operator!=(const LaneOffset& other) const { return !(*this == other); }

}  // namespace xodr
}  // namespace malidrive
