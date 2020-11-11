// Copyright 2020 Toyota Research Institute
#pragma once

#include <map>
#include <memory>
#include <vector>

#include "maliput/geometry_base/junction.h"
#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/base/segment.h"
#include "maliput_malidrive/base/world_to_opendrive_transform.h"
#include "maliput_malidrive/builder/builder_tools.h"
#include "maliput_malidrive/builder/id_providers.h"
#include "maliput_malidrive/builder/road_curve_factory.h"
#include "maliput_malidrive/builder/road_geometry_builder_base.h"
#include "maliput_malidrive/builder/road_geometry_configuration.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace builder {

/// Builder class on top of the `xodr::DBManager` which should
/// already have loaded the map. It will construct a RoadGeometry that maps to
/// the XODR description.
class RoadGeometryBuilder : public RoadGeometryBuilderBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometryBuilder)

  RoadGeometryBuilder() = delete;

  /// Builds a RoadGeometry using malidrive2 backend.
  ///
  /// `manager` must not be nullptr. Ownership will be transferred to the
  /// resulting RoadGeometry.
  ///
  /// `factory` must not be nullptr. Used to create the road curve for the lanes.
  ///
  /// @throws maliput::common::assertion_error When `manager` or `factory` are
  /// nullptr.
  /// @see RoadGeometryBuilderBase::RoadGeometryBuilderBase() for further details.
  RoadGeometryBuilder(std::unique_ptr<xodr::DBManager> manager,
                      const RoadGeometryConfiguration& road_geometry_configuration,
                      const WorldToOpenDriveTransform& world_transform, std::unique_ptr<RoadCurveFactoryBase> factory);

  /// Creates a maliput equivalent backend (malidrive::RoadGeometry).
  std::unique_ptr<const maliput::api::RoadGeometry> operator()() override;

 private:
  // Associates a malidrive::Lane to a XODR Lane.
  struct MatchingLanes {
    Lane* malidrive_lane{};
    MalidriveXodrLaneProperties xodr_lane;
  };

  // Builds a RoadCurve.
  //
  // `road_header` contains the geometry description of the curve model.
  // `geometries_to_simplify` contains which geometries can be simplified from `road_header`.
  //
  // @throws maliput::common::assertion_error When the start point of the geometry doesn't match with the start point of
  // the lane section.
  std::unique_ptr<road_curve::RoadCurve> BuildRoadCurve(
      const xodr::RoadHeader& road_header,
      const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify);

  // Builds malidrive::Lanes from the XODR `lanes` and adds them to `segment`.
  // `xodr_track_id` is road's ID, `xodr_lane_section_index` is the index of
  // the LaneSection within the road.
  // `lanes` must be sorted increasing Xodr Lane's ID.
  // It is expected to have right and left lanes at a `lane_section`.
  // When passing right lanes, set `reverse_build` to `true` to correctly
  // build Lanes and add them to the `segment`.
  //
  // `segment` must not be nullptr.
  // `lane_section` must not be nullptr.
  // `road_header` must not be nullptr.
  // `rg` must not be nullptr.
  //
  // @throws maliput::common::assertion_error When either `segment`,
  //         `lane_section`, `road_header` or `rg` are nullptr.
  void BuildLanesForSegment(const std::vector<xodr::Lane>& lanes, int xodr_lane_section_index,
                            const xodr::LaneSection* lane_section, const xodr::RoadHeader* road_header,
                            bool reverse_build, Segment* segment, RoadGeometry* rg);

  // Returns a maliput::geometry_base::Junction whose ID will be "`xodr_track_id`_`lane_section_index`.
  //
  // `xodr_track_id` must be non-negative number.
  // `lane_section_index` must be non negative.
  //
  // @throws maliput::common::assertion_error When `xodr_track_id` holds a negative number.
  // @throws maliput::common::assertion_error When `lane_section_index` is negative.
  std::unique_ptr<maliput::geometry_base::Junction> BuildJunction(const std::string& xodr_track_id,
                                                                  int lane_section_index);

  // Returns a maliput::geometry_base::Junction whose ID will be `xodr_junction_id`.
  //
  // `xodr_junction_id` must be non-negative number.
  //
  // @throws maliput::common::assertion_error When `xodr_junction_id` is
  //         negative.
  std::unique_ptr<maliput::geometry_base::Junction> BuildJunction(const std::string& xodr_junction_id);

  // Returns a ground curve built from `geometry`.
  //
  // `geometries` Is a vector of xodr::Geometry. It must not be empty.
  // `geometries_to_simplify` contains which geometries can be simplified from `geometries`.
  //
  // @throws maliput::common::assertion_error When `geometries` is empty.
  // @throws maliput::common::assertion_error When any item in `geometries` has an unsupported type.
  std::unique_ptr<malidrive::road_curve::GroundCurve> MakeGroundCurve(
      const std::vector<xodr::Geometry>& geometries,
      const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify);

  // Finds or creates for each lane a BranchPoint for each end.
  //
  // `rg` must not be nullptr.
  //
  // @throws maliput::common::assertion_error When `rg` is nullptr.
  void BuildBranchPointsForLanes(RoadGeometry* rg);

  // Finds or creates for `lane` a BranchPoint for each end.
  //
  // `lane` must not be nullptr.
  // `rg` must not be nullptr.
  //
  // @throws maliput::common::assertion_error When either `lane` or `rg` are
  //         nullptr.
  void FindOrCreateBranchPointFor(const MalidriveXodrLaneProperties& xodr_lane_properties, Lane* lane,
                                  RoadGeometry* rg);

  // Identifies which kind of XODR Lane maps to `lane_end.lane` and then the
  // connection it refers. We can identify the following types:
  //
  // If the road does not belong to a Junction:
  // - Inner LaneSection: XODR Lane would only connect to other lane within the
  //   same road.
  // - Extreme LaneSection: XODR Lane would only connect to other XODR Road, or
  //   Junction.
  // If the road belongs to a Junction:
  // - If the Lane Section is internal, it is treated as before, otherwise, it
  //   checks which is the other Road it connects to.
  //
  // Returns a vector of LaneEnds that connect to `lane_end`.
  //
  // `rg` must not be nullptr.
  //
  // @throws maliput::common::assertion_error When `rg` is nullptr.
  std::vector<maliput::api::LaneEnd> FindConnectingLaneEndsForLaneEnd(
      const maliput::api::LaneEnd& lane_end, const MalidriveXodrLaneProperties& xodr_lane_properties, RoadGeometry* rg);

  const RoadGeometryConfiguration::SimplificationPolicy simplification_policy_{};
  const RoadGeometryConfiguration::ToleranceSelectionPolicy tolerance_selection_policy_{};
  std::unique_ptr<xodr::DBManager> manager_;
  std::unique_ptr<RoadCurveFactoryBase> factory_;
  // Key on LaneId to ensure iteration over this map is deterministic. This
  // ensures default branch point selection is deterministic.
  std::map<maliput::api::LaneId, MatchingLanes> lane_xodr_lane_properties_;
};

}  // namespace builder
}  // namespace malidrive
