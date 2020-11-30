// Copyright 2020 Toyota Research Institute
#pragma once

#include <array>
#include <optional>
#include <string>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/unit.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR description of a type header.
/// For example, a XML node describing a XODR's type:
/// @code{.xml}
///   <OpenDRIVE>
///     ...
///     <road name="Road 0" length="4.9118886948705835e+1" id="0" junction="-1">
///         <type s="0.0000000000000000e+0" type="town">
///             <speed max="40" unit="mph"/>
///         </type>
///           ...
///     </road>
///     ...
///   </OpenDRIVE>
/// @endcode
struct RoadType {
  /// Convenient constants that hold the tag names in the XODR Type header description.
  static constexpr const char* kRoadTypeTag = "type";
  static constexpr const char* kS0 = "s";
  static constexpr const char* kType = "type";
  static constexpr const char* kCountry = "country";

  /// Contains the types of road.
  enum class Type {
    kUnknown = 0,
    kRural,
    kMotorway,
    kTown,
    kLowSpeed,
    kPedestrian,
    kBicycle,
    kTownExpressway,
    kTownCollector,
    kTownArterial,
    kTownPrivate,
    kTownLocal,
    kTownPlayStreet,
  };

  /// Speed description.
  struct Speed {
    /// Holds the tag name in the XODR Speed description.
    static constexpr const char* kSpeedTag = "speed";
    static constexpr const char* kMax = "max";
    static constexpr const char* kUnit = "unit";

    /// Holds possible string values for `max` attribute.
    static constexpr std::array<const char*, 2> kUnlimitedSpeedStrings{"no limit", "undefined"};

    /// Maximum allowed speed.
    /// It contains `std::nullopt` when there is no limit.
    std::optional<double> max{std::nullopt};

    /// Unit of the `max` attribute.
    Unit unit{Unit::kMs};

    /// Equality operator.
    bool operator==(const Speed& other) const { return max == other.max && unit == other.unit; }

    /// Inequality operator.
    bool operator!=(const Speed& other) const { return !(*this == other); }
  };

  /// Matches string with a Type.
  /// @param type Is a Type.
  /// @returns A string that matches with `type`.
  static std::string type_to_str(Type type);

  /// Matches Type with a string.
  /// @param type Is a string.
  /// @returns A Type that matches with `type`.
  /// @throw maliput::common::assertion_error When `type` doesn't match with a Type.
  static Type str_to_type(const std::string& type);

  /// Equality operator.
  bool operator==(const RoadType& other) const;

  /// Inequality operator.
  bool operator!=(const RoadType& other) const;

  /// Start position (s-coordinate).
  double s_0{};
  /// Type of the road.
  Type type{Type::kUnknown};
  /// Country code of the road.
  std::optional<std::string> country{std::nullopt};
  /// Maximum speed allowed.
  Speed speed{};
};

}  // namespace xodr
}  // namespace malidrive
