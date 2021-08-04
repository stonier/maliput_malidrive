// Copyright 2020 Toyota Research Institute
#pragma once

#include <map>
#include <memory>
#include <string>

#include <maliput/api/road_network.h>

#include "maliput_malidrive/builder/road_network_builder_base.h"

namespace malidrive {
namespace builder {

class RoadNetworkBuilder : public RoadNetworkBuilderBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadNetworkBuilder);

  /// Constructs a RoadNetworkBuilder.
  ///
  /// @param road_network_configuration Holds the information of all the
  ///        RoadNetwork entities.
  RoadNetworkBuilder(const std::map<std::string, std::string>& road_network_configuration)
      : RoadNetworkBuilderBase(road_network_configuration) {}

  /// TODO(#135): Deprecates this Constructor.
  ///
  /// Constructs a RoadNetworkBuilder.
  ///
  /// @param road_network_configuration Holds the information of all the
  ///        RoadNetwork entities.
  RoadNetworkBuilder(const RoadNetworkConfiguration& road_network_configuration)
      : RoadNetworkBuilder(road_network_configuration.ToStringMap()) {}

  /// @return A malidrive RoadNetwork without OpenDrive SDK.
  std::unique_ptr<maliput::api::RoadNetwork> operator()() const override;
};

}  // namespace builder
}  // namespace malidrive
