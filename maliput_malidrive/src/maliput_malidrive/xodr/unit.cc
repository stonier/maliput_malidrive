// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/unit.h"

#include <map>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {
namespace {

// Map for Unit to string conversion.
const std::map<Unit, std::string> unit_to_str_map{{Unit::kMs, "m/s"}, {Unit::kMph, "mph"}, {Unit::kKph, "km/h"}};

// Map for string to Unit conversion.
const std::map<std::string, Unit> str_to_unit_map{{"m/s", Unit::kMs}, {"mph", Unit::kMph}, {"km/h", Unit::kKph}};

}  // namespace

std::string unit_to_str(Unit unit) { return unit_to_str_map.at(unit); }

Unit str_to_unit(const std::string& unit) {
  if (str_to_unit_map.find(unit) == str_to_unit_map.end()) {
    MALIDRIVE_THROW_MESSAGE(unit + " unit type is not available.");
  }
  return str_to_unit_map.at(unit);
}

double ConvertToMs(double value, Unit unit) {
  const double kKphFactor{10. / 36.};
  const double kMphFactor{1609.344 / 3600.};
  switch (unit) {
    case Unit::kKph:
      return value * kKphFactor;
    case Unit::kMph:
      return value * kMphFactor;
    case Unit::kMs:
      return value;
    default:
      MALIDRIVE_THROW_MESSAGE("This unit conversion is not supported.");
  }
}

}  // namespace xodr
}  // namespace malidrive
