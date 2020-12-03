// Copyright 2019 Toyota Research Institute
#pragma once

#include <optional>
#include <string>

#include "maliput_malidrive/builder/road_geometry_configuration.h"

namespace malidrive {
namespace builder {

/// Structure to hold file paths for multiple RoadNetwork structures.
struct RoadNetworkConfiguration {
  /// Road geometry configuration parameters.
  const RoadGeometryConfiguration road_geometry_configuration;
  /// Path to the configuration file to load a RoadRulebook
  std::optional<std::string> road_rule_book{};
  /// Path to the configuration file to load a TrafficLightBook.
  std::optional<std::string> traffic_light_book{};
  /// Path to the configuration file to load a PhaseRingBook.
  std::optional<std::string> phase_ring_book{};
  /// Path to the configuration file to load an IntersectionBook.
  std::optional<std::string> intersection_book{};
};

}  // namespace builder
}  // namespace malidrive
