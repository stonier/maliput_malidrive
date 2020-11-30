// Copyright 2020 Toyota Research Institute
#pragma once

#include <optional>
#include <ostream>
#include <string>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR description's header.
/// For example, a XML node describing a XODR's header:
/// @code{.xml}
///   <OpenDRIVE>
///       <header revMajor="1" revMinor="1" name="ExampleMap" version="1.00" date="Fri May 08 12:00:00 2020"
///       north="0.0000000000000000e+00" south="0.0000000000000000e+00" east="0.0000000000000000e+00"
///       west="0.0000000000000000e+00" vendor="VendorName">
///       </header>
///       ...
///       ...
///   </OpenDRIVE>
/// @endcode
struct Header {
  /// Convenient constants that hold the tag names in the XODR header description.
  static constexpr const char* kHeaderTag = "header";
  static constexpr const char* kXodrRevMajor = "revMajor";
  static constexpr const char* kXodrRevMinor = "revMinor";
  static constexpr const char* kXodrName = "name";
  static constexpr const char* kXodrVersion = "version";
  static constexpr const char* kXodrDate = "date";
  static constexpr const char* kXodrNorth = "north";
  static constexpr const char* kXodrSouth = "south";
  static constexpr const char* kXodrEast = "east";
  static constexpr const char* kXodrWest = "west";
  static constexpr const char* kXodrVendor = "vendor";

  /// Equality operator.
  bool operator==(const Header& other) const;

  /// Inequality operator.
  bool operator!=(const Header& other) const;

  /// Major revision number of Malidrive format.
  double rev_major{};
  /// Minor revision number of Malidrive format.
  double rev_minor{};
  /// Database name.
  std::optional<std::string> name{std::nullopt};
  /// Database's version number.
  std::optional<double> version{std::nullopt};
  /// Time/date of database creation according to ISO 8601.
  /// TODO(francocipollone): Use std::chrono::system_clock::time_point instead of std::string.
  std::optional<std::string> date{std::nullopt};
  /// Maximum inertial `y` value.
  std::optional<double> north{std::nullopt};
  /// Minimum inertial `y` value.
  std::optional<double> south{std::nullopt};
  /// Maximum inertial `x` value.
  std::optional<double> east{std::nullopt};
  /// Minimum inertial `x` value.
  std::optional<double> west{std::nullopt};
  /// Vendor name.
  std::optional<std::string> vendor{std::nullopt};
};

/// Streams a string representation of @p header into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const Header& header);

}  // namespace xodr
}  // namespace malidrive
