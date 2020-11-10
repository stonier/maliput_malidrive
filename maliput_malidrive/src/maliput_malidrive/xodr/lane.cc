// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lane.h"

namespace malidrive {
namespace xodr {
namespace {

const std::map<std::string, Lane::Type> str_to_type_map{
    {"none", Lane::Type::kNone},
    {"driving", Lane::Type::kDriving},
    {"stop", Lane::Type::kStop},
    {"shoulder", Lane::Type::kShoulder},
    {"biking", Lane::Type::kBiking},
    {"sidewalk", Lane::Type::kSidewalk},
    {"border", Lane::Type::kBorder},
    {"restricted", Lane::Type::kRestricted},
    {"parking", Lane::Type::kParking},
    {"bidirectional", Lane::Type::kBidirectional},
    {"median", Lane::Type::kMedian},
    {"special1", Lane::Type::kSpecial1},
    {"special2", Lane::Type::kSpecial2},
    {"special3", Lane::Type::kSpecial3},
    {"roadworks", Lane::Type::kRoadWorks},
    {"tram", Lane::Type::kTram},
    {"rail", Lane::Type::kRail},
    {"entry", Lane::Type::kEntry},
    {"exit", Lane::Type::kExit},
    {"offRamp", Lane::Type::kOffRamp},
    {"onRamp", Lane::Type::kOnRamp},
    {"connectingRamp", Lane::Type::kConnectingRamp},
    {"bus", Lane::Type::kBus},
    {"taxi", Lane::Type::kTaxi},
    {"hov", Lane::Type::kHOV},
    {"mwyEntry", Lane::Type::kMwyEntry},
    {"mwyExit", Lane::Type::kMwyExit},
};

const std::map<Lane::Type, std::string> type_to_str_map{
    {Lane::Type::kNone, "none"},
    {Lane::Type::kDriving, "driving"},
    {Lane::Type::kStop, "stop"},
    {Lane::Type::kShoulder, "shoulder"},
    {Lane::Type::kBiking, "biking"},
    {Lane::Type::kSidewalk, "sidewalk"},
    {Lane::Type::kBorder, "border"},
    {Lane::Type::kRestricted, "restricted"},
    {Lane::Type::kParking, "parking"},
    {Lane::Type::kBidirectional, "bidirectional"},
    {Lane::Type::kMedian, "median"},
    {Lane::Type::kSpecial1, "special1"},
    {Lane::Type::kSpecial2, "special2"},
    {Lane::Type::kSpecial3, "special3"},
    {Lane::Type::kRoadWorks, "roadworks"},
    {Lane::Type::kTram, "tram"},
    {Lane::Type::kRail, "rail"},
    {Lane::Type::kEntry, "entry"},
    {Lane::Type::kExit, "exit"},
    {Lane::Type::kOffRamp, "offRamp"},
    {Lane::Type::kOnRamp, "onRamp"},
    {Lane::Type::kConnectingRamp, "connectingRamp"},
    {Lane::Type::kBus, "bus"},
    {Lane::Type::kTaxi, "taxi"},
    {Lane::Type::kHOV, "hov"},
    {Lane::Type::kMwyEntry, "mwyEntry"},
    {Lane::Type::kMwyExit, "mwyExit"},
};

}  // namespace

std::string Lane::type_to_str(Type type) { return type_to_str_map.at(type); }

Lane::Type Lane::str_to_type(const std::string& type) {
  if (str_to_type_map.find(type) == str_to_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " lane type is not available.");
  }
  return str_to_type_map.at(type);
}

bool Lane::operator==(const Lane& other) const {
  return id == other.id && type == other.type && level == other.level && lane_link == other.lane_link &&
         width_description == other.width_description && speed == other.speed && user_data == other.user_data;
}

bool Lane::operator!=(const Lane& other) const { return !(*this == other); }

}  // namespace xodr
}  // namespace malidrive
