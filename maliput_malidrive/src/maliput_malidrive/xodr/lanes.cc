// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lanes.h"

namespace malidrive {
namespace xodr {

bool Lanes::operator==(const Lanes& other) const {
  return lanes_offset == other.lanes_offset && lanes_section == other.lanes_section;
}

bool Lanes::operator!=(const Lanes& other) const { return !(*this == other); }

}  // namespace xodr
}  // namespace malidrive
