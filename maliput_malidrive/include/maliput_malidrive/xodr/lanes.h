// Copyright 2020 Toyota Research Institute
#pragma once

#include <optional>
#include <vector>

#include "maliput_malidrive/xodr/lane_offset.h"
#include "maliput_malidrive/xodr/lane_section.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR Lanes node.
/// For example, a XML node describing a XODR's lanes node:
/// @code{.xml}
///  <OpenDRIVE>
///       ...
///   <lanes>
///      <laneOffset s="0.0000000000000000e+0" a="-4.000000000e+0" b="0.000000000e+0" c="0.0000000e+0"
///      d="0.0000000e+0"/> <laneOffset s="3.8268524704053952e-2" a="-4.000000000e+0" b="0.000000000e+0"
///      c="0.0000000e+0" d="0.0000000e+0"/> <laneSection s="0.0000000000000000e+0">
///        ...
///      </laneSection>
///      <laneSection s="3.8268524704053952e-2">
///        ...
///      </laneSection>
///        ...
///   </lanes>
///     ...
///  </OpenDRIVE>
/// @endcode
struct Lanes {
  /// Hold the tag for the lanes in the XODR lanes description.
  static constexpr const char* kLanesTag = "lanes";

  /// Equality operator.
  bool operator==(const Lanes& other) const;

  /// Inequality operator.
  bool operator!=(const Lanes& other) const;

  /// Holds all the `LaneOffset`s in the road.
  std::vector<LaneOffset> lanes_offset{};
  /// Holds the `LaneSection`s that there are in the road.
  std::vector<LaneSection> lanes_section{};
};

}  // namespace xodr
}  // namespace malidrive
