// Copyright 2020 Toyota Research Institute
#pragma once

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR Lane Offset.
/// For example, a XML node describing a XODR's lane offset:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///     <road>
///       <lanes>
///         <laneOffset s="0.000e+0" a="0.000e+0" b="0.000e+0" c="0.000e+0" d="0.000e+0"/>
///           ...
///       </lanes>
///     <road>
///       ...
///   </OpenDRIVE>
/// @endcode
struct LaneOffset {
  /// Convenient constants that hold the tag names in the XODR lane offset description.
  static constexpr const char* kLaneOffsetTag = "laneOffset";
  static constexpr const char* kS0 = "s";
  static constexpr const char* kA = "a";
  static constexpr const char* kB = "b";
  static constexpr const char* kC = "c";
  static constexpr const char* kD = "d";

  /// Equality operator.
  bool operator==(const LaneOffset& other) const;

  /// Inequality operator.
  bool operator!=(const LaneOffset& other) const;

  /// Start position (s-coordinate).
  double s_0{};
  /// Coefficients of a cubic polynomial: @f$a + b * p + c * p^2 + d * p^3@f$.
  /// @{
  double a{};
  double b{};
  double c{};
  double d{};
  /// @}
};

}  // namespace xodr
}  // namespace malidrive
