// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/road_curve/open_range_validator.h"

#include <algorithm>

namespace malidrive {
namespace road_curve {

double OpenRangeValidator::operator()(double s) const {
  MALIDRIVE_IS_IN_RANGE(s, min_ - tolerance_, max_ + tolerance_);
  return std::clamp(s, min_ + epsilon_, max_ - epsilon_);
}

}  // namespace road_curve
}  // namespace malidrive
