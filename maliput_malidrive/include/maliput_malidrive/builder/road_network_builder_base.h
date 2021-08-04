// Copyright 2020 Toyota Research Institute
#pragma once

#include <map>
#include <memory>
#include <string>

#include <maliput/api/road_network.h>

#include "maliput_malidrive/builder/road_network_configuration.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Interface to build a RoadNetwork with a malidrive backend.
class RoadNetworkBuilderBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadNetworkBuilderBase);
  virtual ~RoadNetworkBuilderBase() = default;

  /// Constructs a RoadNetworkBuilderBase.
  ///
  /// @param road_network_configuration Holds the information of all the
  ///        RoadNetwork entities.
  RoadNetworkBuilderBase(const std::map<std::string, std::string>& road_network_configuration)
      : road_network_configuration_(road_network_configuration) {}

  /// TODO(#135): Deprecates this Constructor.
  ///
  /// Constructs a RoadNetworkBuilderBase.
  ///
  /// @param road_network_configuration Holds the information of all the
  ///        RoadNetwork entities.
  RoadNetworkBuilderBase(const RoadNetworkConfiguration& road_network_configuration)
      : road_network_configuration_(road_network_configuration.ToStringMap()) {}

  /// @return A maliput::api::RoadNetwork.
  virtual std::unique_ptr<maliput::api::RoadNetwork> operator()() const = 0;

 protected:
  const std::map<std::string, std::string> road_network_configuration_;
};

}  // namespace builder
}  // namespace malidrive
