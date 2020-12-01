// Copyright 2020 Toyota Research Institute
#pragma once

#include <optional>
#include <vector>

#include "maliput_malidrive/xodr/lane.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR LaneSection.
/// For example, a XML node describing a XODR's lane section:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///     <road>
///       <lanes>
///         <laneSection s="0.0000000000000000e+0" singleSide="true">
///           <left>
///             <lane>
///               ...
///             </lane>
///           </left>
///           <center>
///             <lane>
///               ...
///             </lane>
///           </center>
///           <right>
///             <lane>
///               ...
///             </lane>
///           </right>
///         </laneSection>
///       </lanes>
///     <road>
///       ...
///   </OpenDRIVE>
/// @endcode
struct LaneSection {
  /// Convenient constants that hold the tag names in the XODR lane section node.
  static constexpr const char* kLaneSectionTag = "laneSection";
  static constexpr const char* kS0 = "s";
  static constexpr const char* kSingleSide = "singleSide";
  static constexpr const char* kLeft = "left";
  static constexpr const char* kCenter = "center";
  static constexpr const char* kRight = "right";

  /// Equality operator.
  bool operator==(const LaneSection& other) const;

  /// Inequality operator.
  bool operator!=(const LaneSection& other) const;

  /// Start position (s-coordinate).
  double s_0{};
  /// Lane section entry is valid for one side only.
  std::optional<bool> single_side{std::nullopt};
  /// Lanes in the left side of the road.
  std::vector<Lane> left_lanes{};
  /// Center lane of the road.
  Lane center_lane{};
  /// Lanes in the right side of the road.
  std::vector<Lane> right_lanes{};
};

}  // namespace xodr
}  // namespace malidrive
