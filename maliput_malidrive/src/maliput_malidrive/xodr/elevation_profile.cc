// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/elevation_profile.h"

namespace malidrive {
namespace xodr {

bool ElevationProfile::Elevation::operator==(const ElevationProfile::Elevation& other) const {
  return s_0 == other.s_0 && a == other.a && b == other.b && c == other.c && d == other.d;
}

bool ElevationProfile::Elevation::operator!=(const ElevationProfile::Elevation& other) const {
  return !(*this == other);
}

}  // namespace xodr
}  // namespace malidrive
