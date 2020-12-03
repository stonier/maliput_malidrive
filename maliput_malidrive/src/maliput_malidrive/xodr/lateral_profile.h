// Copyright 2020 Toyota Research Institute
#pragma once

#include <vector>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

/// Contains a series of superelevation records which define the characteristics of
/// the road's lateral profile along the reference line.
/// For example, a XML node describing a lateralProfile header:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///         <lateralProfile>
///             <superelevation s="1.12459e+1" a="3.037354772e+0" b="-3.505664e-3" c="-3.38446126e-5" d="0.00000e+0"/>
///             <superelevation s="2.12459e+1" a="4.037354772e+0" b="-5.505664e-5" c="-6.38446126e-5" d="1.00000e+0"/>
///             ...
///         </lateralProfile>
///       ...
///   </OpenDRIVE>
/// @endcode
struct LateralProfile {
  /// The superelevation record defines an superelevation entry at a given reference line position.
  struct Superelevation {
    /// Convenient constants that hold the tag names in the XODR superelevation description.
    static constexpr const char* kSuperelevationTag = "superelevation";
    static constexpr const char* kS0 = "s";
    static constexpr const char* kA = "a";
    static constexpr const char* kB = "b";
    static constexpr const char* kC = "c";
    static constexpr const char* kD = "d";
    /// Equality operator.
    bool operator==(const Superelevation& other) const;

    /// Inequality operator.
    bool operator!=(const Superelevation& other) const;

    /// Start position (s-coordinate).
    double s_0{};
    /// Coefficients of a cubic polynomial: @f$ a + b * p + c * p^2 + d * p^3 @f$.
    /// @{
    double a{};
    double b{};
    double c{};
    double d{};
    /// @}
  };
  static constexpr const char* kLateralProfileTag = "lateralProfile";

  /// Equality operator.
  bool operator==(const LateralProfile& other) const { return superelevations == other.superelevations; }

  /// Inequality operator.
  bool operator!=(const LateralProfile& other) const { return superelevations != other.superelevations; }

  /// Contains the Superelevations that describe the lateral profile of a Road.
  std::vector<Superelevation> superelevations{};
};

}  // namespace xodr
}  // namespace malidrive
