// Copyright 2020 Toyota Research Institute
#pragma once

#include <vector>

#include "maliput_malidrive/xodr/db_manager.h"
#include "maliput_malidrive/xodr/geometry.h"

namespace malidrive {
namespace builder {

/// Creates a new vector of xodr::Geometry out of `geometries` by simplifying
/// it following `geometries_to_simplify`'s actions.
///
/// @param geometries The vector of xodr::Geometry to simplify of the same
///        RoadHeader::Id.
/// @param geometries_to_simplify Contains disjunct groups of geometry indices
///        with the same xodr::RoadHeader::Id. When it is empty, no action is
///        performed.
/// @return A vector of xodr:Geometry.
std::vector<xodr::Geometry> SimplifyGeometries(
    const std::vector<xodr::Geometry>& geometries,
    const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify);

}  // namespace builder
}  // namespace malidrive
