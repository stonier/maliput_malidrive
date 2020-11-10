// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/junction.h"

#include <map>

namespace malidrive {
namespace xodr {
namespace {

const std::map<std::string, Junction::Type> str_to_type_map{{"default", Junction::Type::kDefault},
                                                            {"virtual", Junction::Type::kVirtual}};

const std::map<Junction::Type, std::string> type_to_str_map{{Junction::Type::kDefault, "default"},
                                                            {Junction::Type::kVirtual, "virtual"}};

}  // namespace

Junction::Type Junction::str_to_type(const std::string& type) {
  if (str_to_type_map.find(type) == str_to_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " type is not available.");
  }
  return str_to_type_map.at(type);
}

std::string Junction::type_to_str(Type type) { return type_to_str_map.at(type); }

bool Junction::operator==(const Junction& other) const {
  return id == other.id && name == other.name && type == other.type && connections == other.connections;
}

std::ostream& operator<<(std::ostream& out, const Junction& junction) {
  out << "{\"id\": " << junction.id.string();
  out << ", \"name\": " << (junction.name.has_value() ? junction.name.value() : "");
  out << ", \"type\": {" << (junction.type.has_value() ? Junction::type_to_str(junction.type.value()) : "") << "}";
  out << "}";
  // TODO(malidrive#574): Once other entities acquire serialization overloads, we should include them.
  return out;
}

}  // namespace xodr
}  // namespace malidrive
