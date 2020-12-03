// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lane_width.h"

namespace malidrive {
namespace xodr {

bool LaneWidth::operator==(const LaneWidth& other) const {
  return offset == other.offset && a == other.a && b == other.b && c == other.c && d == other.d;
}

bool LaneWidth::operator!=(const LaneWidth& other) const { return !(*this == other); }

}  // namespace xodr
}  // namespace malidrive
