// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/road_link.h"

#include <map>

namespace malidrive {
namespace xodr {
namespace {

const std::map<std::string, RoadLink::ElementType> str_to_element_type_map{
    {"road", RoadLink::ElementType::kRoad}, {"junction", RoadLink::ElementType::kJunction}};

const std::map<RoadLink::ElementType, std::string> element_type_to_str_map{
    {RoadLink::ElementType::kRoad, "road"}, {RoadLink::ElementType::kJunction, "junction"}};

const std::map<std::string, RoadLink::ContactPoint> str_to_contact_point_map{{"start", RoadLink::ContactPoint::kStart},
                                                                             {"end", RoadLink::ContactPoint::kEnd}};

const std::map<RoadLink::ContactPoint, std::string> contact_point_to_str_map{{RoadLink::ContactPoint::kStart, "start"},
                                                                             {RoadLink::ContactPoint::kEnd, "end"}};

}  // namespace

RoadLink::ElementType RoadLink::str_to_element_type(const std::string& type) {
  if (str_to_element_type_map.find(type) == str_to_element_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " element type is not available.");
  }
  return str_to_element_type_map.at(type);
}

std::string RoadLink::element_type_to_str(ElementType type) { return element_type_to_str_map.at(type); }

RoadLink::ContactPoint RoadLink::str_to_contact_point(const std::string& contact_point) {
  if (str_to_contact_point_map.find(contact_point) == str_to_contact_point_map.end()) {
    MALIDRIVE_THROW_MESSAGE(contact_point + " contact_point is not available.");
  }
  return str_to_contact_point_map.at(contact_point);
}

std::string RoadLink::contact_point_to_str(ContactPoint contact_point) {
  return contact_point_to_str_map.at(contact_point);
}

bool RoadLink::LinkAttributes::operator==(const LinkAttributes& other) const {
  return element_type == other.element_type && element_id == other.element_id && contact_point == other.contact_point;
}

}  // namespace xodr
}  // namespace malidrive
