// Copyright 2020 Toyota Research Institute
#pragma once

#include <memory>

#include "maliput_malidrive/base/world_to_opendrive_transform.h"
#include "maliput_malidrive/builder/road_network_builder_base.h"
#include "maliput_malidrive/builder/road_network_configuration.h"

#include "maliput/api/road_network.h"

namespace malidrive {
namespace builder {

class MalidriveRoadNetworkBuilder : public RoadNetworkBuilderBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(MalidriveRoadNetworkBuilder);

  /// Constructs a MalidriveRoadNetworkBuilder.
  ///
  /// @param road_network_configuration Holds the information of all the
  ///        RoadNetwork entities.
  /// @param world_transform Translation from the Inertial World frame to the
  ///        RoadGeometry Inertial Frame.
  MalidriveRoadNetworkBuilder(const RoadNetworkConfiguration& road_network_configuration,
                              const WorldToOpenDriveTransform world_transform)
      : RoadNetworkBuilderBase(road_network_configuration, world_transform) {}

  /// @return A malidrive RoadNetwork without OpenDrive SDK.
  std::unique_ptr<maliput::api::RoadNetwork> operator()() const override;
};

}  // namespace builder
}  // namespace malidrive
