// Copyright 2020 Toyota Research Institute
#pragma once

#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace builder {

static constexpr double kMinLinearTolerance{1e-3};

/// Determines the runtime tolerance a RoadGeometry should be set from the
/// characteristics in a XODR map.
///
/// The selection is based on two constraints:
/// - tolerance must be bigger than the largest gap --> hard constraint
/// - tolerance is desired to be smaller than the smallest geometry piece -->
///   soft constraint.
///
/// The hard constraint is required because that would make the builder or the
/// RoadGeometry itself to throw because of not meeting the G1 constraint.
/// The soft constraint is desired when the previous cannot be used. However,
/// because synthesized maps might have relatively small road descriptions which
/// are several order of magnitude less than the road dimension the tolerance
/// is saturated between kMinLinearTolerance and constants::kLinearTolerance.
///
/// The tolerance assignment is done as follows:
/// - The maximum between the largest geometry gap and elevation gap is
///   selected and inflated a 50% to avoid any tolerance issue. The value is
///   lower bound saturated with kMinLinearTolerance. If any of the magnitudes
///   is not valid the other is used. When none are used, the soft constraint is
///   used.
/// - The minimum between the geometries and the lane section extension is used.
///   The value is reduced to a half to avoid any numerical discrepancy and
///   saturated between kMinLinearTolerance and constants::kLinearTolerance.
///
/// @param xodr_manager A pointer to a xodr::DBManager with a XODR map loaded.
/// @return A proposed linear tolerance.
/// @throws maliput::common::assertion_error When `xodr_manager` is nullptr.
double DetermineRoadGeometryLinearTolerance(const xodr::DBManager* xodr_manager);

/// Determines the runtime angular a RoadGeometry should be set from the
/// characteristics in a XODR map.
///
/// @param xodr_manager A pointer to a xodr::DBManager with a XODR map loaded.
/// @return The proposed angular tolerance.
/// @throws maliput::common::assertion_error When `xodr_manager` is nullptr.
// TODO(#571): Implement constraints to determine the angular tolerance.
double DetermineRoadGeometryAngularTolerance(const xodr::DBManager* xodr_manager);

/// Determines the runtime scale length a RoadGeometry should be set from the
/// characteristics in a XODR map.
///
/// @param xodr_manager A pointer to a xodr::DBManager with a XODR map loaded.
/// @param linear_tolerance The proposed linear tolerance. It must be positive.
/// @param angular_tolerance The proposed angular tolerance. It must be positive.
/// @return The proposed scale length.
/// @throws maliput::common::assertion_error When `xodr_manager` is nullptr.
// TODO(#571): Implement constraints to determine the scale length.
double DetermineRoadGeometryScaleLength(const xodr::DBManager* xodr_manager, double linear_tolerance,
                                        double angular_tolerance);

}  // namespace builder
}  // namespace malidrive
