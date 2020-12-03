// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/connection.h"

#include <map>

namespace malidrive {
namespace xodr {
namespace {

const std::map<std::string, Connection::Type> str_to_type_map{{"default", Connection::Type::kDefault},
                                                              {"virtual", Connection::Type::kVirtual}};

const std::map<Connection::Type, std::string> type_to_str_map{{Connection::Type::kDefault, "default"},
                                                              {Connection::Type::kVirtual, "virtual"}};

const std::map<std::string, Connection::ContactPoint> str_to_contact_point_map{
    {"start", Connection::ContactPoint::kStart}, {"end", Connection::ContactPoint::kEnd}};

const std::map<Connection::ContactPoint, std::string> contact_point_to_str_map{
    {Connection::ContactPoint::kStart, "start"}, {Connection::ContactPoint::kEnd, "end"}};

}  // namespace

Connection::Type Connection::str_to_type(const std::string& type) {
  if (str_to_type_map.find(type) == str_to_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " type is not available.");
  }
  return str_to_type_map.at(type);
}

std::string Connection::type_to_str(Type type) { return type_to_str_map.at(type); }

Connection::ContactPoint Connection::str_to_contact_point(const std::string& contact_point) {
  if (str_to_contact_point_map.find(contact_point) == str_to_contact_point_map.end()) {
    MALIDRIVE_THROW_MESSAGE(contact_point + " contact point is not available.");
  }
  return str_to_contact_point_map.at(contact_point);
}

std::string Connection::contact_point_to_str(ContactPoint contact_point) {
  return contact_point_to_str_map.at(contact_point);
}

bool Connection::operator==(const Connection& other) const {
  return id == other.id && incoming_road == other.incoming_road && connecting_road == other.connecting_road &&
         contact_point == other.contact_point && connection_master == other.connection_master && type == other.type &&
         lane_links == other.lane_links;
}

}  // namespace xodr
}  // namespace malidrive
