// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/road_type.h"

#include <map>
#include <string>

namespace malidrive {
namespace xodr {
namespace {

// Map for RoadType::Type to string conversion.
const std::map<RoadType::Type, std::string> type_to_str_map{{RoadType::Type::kUnknown, "unknown"},
                                                            {RoadType::Type::kRural, "rural"},
                                                            {RoadType::Type::kMotorway, "motorway"},
                                                            {RoadType::Type::kTown, "town"},
                                                            {RoadType::Type::kLowSpeed, "lowSpeed"},
                                                            {RoadType::Type::kPedestrian, "pedestrian"},
                                                            {RoadType::Type::kBicycle, "bicycle"},
                                                            {RoadType::Type::kTownExpressway, "townExpressway"},
                                                            {RoadType::Type::kTownCollector, "townCollector"},
                                                            {RoadType::Type::kTownArterial, "townArterial"},
                                                            {RoadType::Type::kTownPrivate, "townPrivate"},
                                                            {RoadType::Type::kTownLocal, "townLocal"},
                                                            {RoadType::Type::kTownPlayStreet, "townPlayStreet"}};

// Map for string to RoadType::Type conversion.
const std::map<std::string, RoadType::Type> str_to_type_map{{"unknown", RoadType::Type::kUnknown},
                                                            {"rural", RoadType::Type::kRural},
                                                            {"motorway", RoadType::Type::kMotorway},
                                                            {"town", RoadType::Type::kTown},
                                                            {"lowSpeed", RoadType::Type::kLowSpeed},
                                                            {"pedestrian", RoadType::Type::kPedestrian},
                                                            {"bicycle", RoadType::Type::kBicycle},
                                                            {"townExpressway", RoadType::Type::kTownExpressway},
                                                            {"townCollector", RoadType::Type::kTownCollector},
                                                            {"townArterial", RoadType::Type::kTownArterial},
                                                            {"townPrivate", RoadType::Type::kTownPrivate},
                                                            {"townLocal", RoadType::Type::kTownLocal},
                                                            {"townPlayStreet", RoadType::Type::kTownPlayStreet}};

}  // namespace

std::string RoadType::type_to_str(RoadType::Type type) { return type_to_str_map.at(type); }

RoadType::Type RoadType::str_to_type(const std::string& type) {
  if (str_to_type_map.find(type) == str_to_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " type is not available.");
  }
  return str_to_type_map.at(type);
}

bool RoadType::operator==(const RoadType& other) const {
  return s_0 == other.s_0 && type == other.type && country == other.country && speed == other.speed;
}

bool RoadType::operator!=(const RoadType& other) const { return !(*this == other); }

}  // namespace xodr
}  // namespace malidrive
