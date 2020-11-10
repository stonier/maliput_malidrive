// Copyright 2020 Toyota Research Institute
#pragma once

#include <string>

namespace malidrive {
namespace xodr {

/// Enumerates the units allowed in the XODR description.
enum class Unit {
  // Meters per second.
  kMs = 0,
  // Miles per hour.
  kMph,
  // Kilometers per hour.
  kKph,
};

/// Convert to meters per second.
/// @param value Is the number to be converted.
/// @param unit Is the unit of `value`.
/// @returns The equivalent in meters per second.
/// @throw maliput::common::assertion_error When `unit` isn't Unit::kMs, Unit::kMph or Unit::kph.
double ConvertToMs(double value, Unit unit);

/// Matches string with a Unit.
/// @param unit Is a Unit.
/// @returns A string that matches with `unit`.
std::string unit_to_str(Unit unit);

/// Matches Unit with a string.
/// @param unit Is a string.
/// @returns A Unit that matches with `unit`.
/// @throw maliput::common::assertion_error When `unit` doesn't match with a Unit.
Unit str_to_unit(const std::string& unit);

}  // namespace xodr
}  // namespace malidrive
