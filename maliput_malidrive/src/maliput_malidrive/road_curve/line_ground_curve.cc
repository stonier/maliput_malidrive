// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/road_curve/line_ground_curve.h"

#include <maliput/math/saturate.h>

namespace malidrive {
namespace road_curve {

maliput::math::Vector2 LineGroundCurve::DoG(double p) const {
  p = validate_p_(p);
  return xy0_ + (p - p0_) / (p1_ - p0_) * dxy_;
}

maliput::math::Vector2 LineGroundCurve::DoGDot(double p) const {
  validate_p_(p);
  return dxy_ / (p1_ - p0_);
}

double LineGroundCurve::DoGInverse(const maliput::math::Vector2& xy) const {
  // unit vector pointing along line from p0 -> p1
  const maliput::math::Vector2& unit_vector = dxy_ / arc_length_;

  const maliput::math::Vector2& xy0_to_xy = xy - xy0_;

  // Compute the distance along the line from the start of the line at p0 to `xy`
  // scaled by the total arc length.
  const double scaled_distance_unsaturated = xy0_to_xy.dot(unit_vector) / arc_length_;
  const double scaled_distance = maliput::math::saturate(scaled_distance_unsaturated, 0., 1.);
  return p0_ + (p1_ - p0_) * scaled_distance;
}

}  // namespace road_curve
}  // namespace malidrive
