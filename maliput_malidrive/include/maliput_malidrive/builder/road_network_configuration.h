// Copyright 2019 Toyota Research Institute
#pragma once

#include <map>
#include <optional>
#include <string>

#include "maliput_malidrive/builder/road_geometry_configuration.h"

namespace malidrive {
namespace builder {

/// Structure to hold file paths for multiple RoadNetwork structures.
struct RoadNetworkConfiguration {
  /// @{ RoadNetwork parameters
  static constexpr char const* kStrRoadRuleBook = "road_rule_book";
  static constexpr char const* kStrTrafficLightBook = "traffic_light_book";
  static constexpr char const* kStrPhaseRingBook = "phase_ring_book";
  static constexpr char const* kStrIntersectionBook = "intersection_book";
  /// @}

  /// Default constructor.
  RoadNetworkConfiguration() = default;

  /// Creates a RoadGeometryConfiguration.
  /// @param road_geometry_configuration RoadGeometryConfiguration
  RoadNetworkConfiguration(const RoadGeometryConfiguration& road_geometry_configuration);

  /// Creates a RoadNetworkConfiguration.
  ///
  /// @param road_geometry_configuration RoadGeometryConfiguration.
  /// @param road_rule_book Path to the Road Rule book file.
  /// @param traffic_light_book Path to the Traffic Light book file.
  /// @param phase_ring_book Path to the Phase Ring book file.
  /// @param intersection_book Path to the Intersection book file.
  RoadNetworkConfiguration(const RoadGeometryConfiguration& road_geometry_configuration,
                           const std::optional<std::string>& road_rule_book,
                           const std::optional<std::string>& traffic_light_book,
                           const std::optional<std::string>& phase_ring_book,
                           const std::optional<std::string>& intersection_book);

  /// Creates a RoadNetworkConfiguration out of a string dictionary which contains parameters to be passed to the
  /// RoadNetworkBuilder.
  /// @param road_network_configuration A string-string map containing the configuration for the builder.
  RoadNetworkConfiguration(const std::map<std::string, std::string>& road_network_configuration);

  /// @returns A string-string map containing the RoadGeometry configuration.
  std::map<std::string, std::string> ToStringMap() const;

  /// Road geometry configuration parameters.
  const RoadGeometryConfiguration road_geometry_configuration;
  /// Path to the configuration file to load a RoadRulebook
  std::optional<std::string> road_rule_book{std::nullopt};
  /// Path to the configuration file to load a TrafficLightBook.
  std::optional<std::string> traffic_light_book{std::nullopt};
  /// Path to the configuration file to load a PhaseRingBook.
  std::optional<std::string> phase_ring_book{std::nullopt};
  /// Path to the configuration file to load an IntersectionBook.
  std::optional<std::string> intersection_book{std::nullopt};
};

}  // namespace builder
}  // namespace malidrive
