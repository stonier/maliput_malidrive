// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/header.h"

namespace malidrive {
namespace xodr {

bool Header::operator==(const Header& other) const {
  return rev_major == other.rev_major && rev_minor == other.rev_minor && name == other.name &&
         version == other.version && date == other.date && north == other.north && south == other.south &&
         east == other.east && west == other.west && vendor == other.vendor;
}

bool Header::operator!=(const Header& other) const { return !(*this == other); }

std::ostream& operator<<(std::ostream& out, const Header& header) {
  out << "{ \"rev_major\": " << header.rev_major << ", \"rev_minor\": " << header.rev_minor;
  out << ", \"name\": {" << (header.name.has_value() ? header.name.value() : "") << "}";
  out << ", \"version\": {" << (header.version.has_value() ? std::to_string(header.version.value()) : "") << "}";
  out << ", \"date\": {" << (header.date.has_value() ? header.date.value() : "") << "}";
  out << ", \"north\": {" << (header.north.has_value() ? std::to_string(header.north.value()) : "") << "}";
  out << ", \"south\": {" << (header.south.has_value() ? std::to_string(header.south.value()) : "") << "}";
  out << ", \"east\": {" << (header.east.has_value() ? std::to_string(header.east.value()) : "") << "}";
  out << ", \"west\": {" << (header.west.has_value() ? std::to_string(header.west.value()) : "") << "}";
  out << ", \"vendor\": {" << (header.vendor.has_value() ? header.vendor.value() : "") << "}";
  out << "}";

  return out;
}

}  // namespace xodr
}  // namespace malidrive
