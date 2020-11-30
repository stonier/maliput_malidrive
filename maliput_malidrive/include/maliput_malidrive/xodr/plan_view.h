// Copyright 2020 Toyota Research Institute
#pragma once

#include <vector>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/geometry.h"

namespace malidrive {
namespace xodr {

/// Holds the plan view geometry description of a XODR road.
/// For example, a XML node describing a XODR's header:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///       <road name="Road 0" length="1.4005927435591335e+2" id="0" junction="-1">
///           <planView>
///               <geometry s="0.0000000000000000e+0" x="0.0000000000000000e+0" y="0.0000000000000000e+0"
///               hdg="1.3734007669450161e+0" length="1.4005927435591335e+2">
///                   <line/>
///               </geometry>
///           </planView>
///           ...
///       </road>
///       ...
///   </OpenDRIVE>
/// @endcode
struct PlanView {
  static constexpr const char* kPlanViewTag = "planView";

  /// Equality operator.
  bool operator==(const PlanView& other) const { return geometries == other.geometries; }

  /// Inequality operator.
  bool operator!=(const PlanView& other) const { return geometries != other.geometries; }

  /// Contains the geometries which define the layout of the road's
  /// reference line in the x/y-plane(plan view).
  std::vector<Geometry> geometries{};
};

}  // namespace xodr
}  // namespace malidrive
