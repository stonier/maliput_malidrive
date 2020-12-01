// Copyright 2020 Toyota Research Institute
#pragma once

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/elevation_profile.h"
#include "maliput_malidrive/xodr/lateral_profile.h"
#include "maliput_malidrive/xodr/plan_view.h"

namespace malidrive {
namespace xodr {

/// Holds the geometry description of a XODR road. It gathers the info from XODR nodes:
/// - `planView`.
/// - `elevationProfile`.
/// - `lateralProfile`.
struct ReferenceGeometry {
  /// Equality operator.
  bool operator==(const ReferenceGeometry& other) const {
    return plan_view == other.plan_view && elevation_profile == other.elevation_profile &&
           lateral_profile == other.lateral_profile;
  }
  /// Inequality operator.
  bool operator!=(const ReferenceGeometry& other) const {
    return plan_view != other.plan_view || elevation_profile != other.elevation_profile ||
           lateral_profile != other.lateral_profile;
  }

  /// Contains the geometries which define the layout of the road's
  /// reference line in the x/y-plane.
  PlanView plan_view{};

  /// Contains a series of elevation records which define the characteristics of
  /// the road's elevation along the reference line.
  ElevationProfile elevation_profile{};

  /// Contains a series of superelevation records which define the characteristics of
  /// the road's lateral profile along the reference line.
  LateralProfile lateral_profile{};
};

}  // namespace xodr
}  // namespace malidrive
