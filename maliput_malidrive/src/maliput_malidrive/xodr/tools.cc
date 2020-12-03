// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/tools.h"

#include <cmath>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/elevation_profile.h"
#include "maliput_malidrive/xodr/lateral_profile.h"

namespace malidrive {
namespace xodr {

using maliput::math::Vector2;

double GetDistanceBetweenGeometries(const Geometry& geometry, const Geometry& rhs) {
  switch (geometry.type) {
    case Geometry::Type::kArc:
    case Geometry::Type::kLine: {
      return (ComputeEndpointWithNewLength(geometry, geometry.length) - rhs.start_point).norm();
    }
    default:
      MALIDRIVE_THROW_MESSAGE("Geometry type could not be recognized.");
  }
}

template <class XodrFunction>
double GetDistanceBetweenFunctions(const XodrFunction& lhs, const XodrFunction& rhs) {
  const double s_0 = rhs.s_0;
  const double s_0_2 = s_0 * s_0;
  const double s_0_3 = s_0 * s_0 * s_0;
  const double lhs_result = lhs.a + lhs.b * s_0 + lhs.c * s_0_2 + lhs.d * s_0_3;
  const double rhs_result = rhs.a + rhs.b * s_0 + rhs.c * s_0_2 + rhs.d * s_0_3;
  return std::abs(lhs_result - rhs_result);
}

Vector2 ComputeEndpointWithNewLength(const Geometry& geometry, double length) {
  MALIPUT_THROW_UNLESS(length > 0.);
  switch (geometry.type) {
    case Geometry::Type::kLine: {
      return geometry.start_point +
             Vector2{std::cos(geometry.orientation) * length, std::sin(geometry.orientation) * length};
    }
    case Geometry::Type::kArc: {
      const double curvature{std::get<xodr::Geometry::Arc>(geometry.description).curvature};
      const double radius{std::abs(1. / curvature)};
      const double d_theta{length * curvature};
      const double theta0 = geometry.orientation - std::copysign(M_PI / 2., curvature);
      const Vector2 center = geometry.start_point - radius * Vector2{std::cos(theta0), std::sin(theta0)};
      const double theta1 = theta0 + d_theta;
      return center + radius * Vector2(std::cos(theta1), std::sin(theta1));
    }
    default:
      MALIDRIVE_THROW_MESSAGE("Geometry type could not be recognized.");
  }
}

template double GetDistanceBetweenFunctions<ElevationProfile::Elevation>(const ElevationProfile::Elevation& lhs,
                                                                         const ElevationProfile::Elevation& rhs);
template double GetDistanceBetweenFunctions<LateralProfile::Superelevation>(const LateralProfile::Superelevation& lhs,
                                                                            const LateralProfile::Superelevation& rhs);

}  // namespace xodr
}  // namespace malidrive
