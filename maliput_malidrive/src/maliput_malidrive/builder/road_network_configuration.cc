// Copyright 2021 Toyota Research Institute

#include "maliput_malidrive/builder/road_network_configuration.h"

namespace malidrive {
namespace builder {

RoadNetworkConfiguration::RoadNetworkConfiguration(const RoadGeometryConfiguration& road_geometry_configuration)
    : road_geometry_configuration(road_geometry_configuration) {}

RoadNetworkConfiguration::RoadNetworkConfiguration(const RoadGeometryConfiguration& road_geometry_configuration,
                                                   const std::optional<std::string>& road_rule_book,
                                                   const std::optional<std::string>& traffic_light_book,
                                                   const std::optional<std::string>& phase_ring_book,
                                                   const std::optional<std::string>& intersection_book)
    : road_geometry_configuration(road_geometry_configuration),
      road_rule_book(road_rule_book),
      traffic_light_book(traffic_light_book),
      phase_ring_book(phase_ring_book),
      intersection_book(intersection_book) {}

RoadNetworkConfiguration::RoadNetworkConfiguration(const std::map<std::string, std::string>& road_network_configuration)
    : road_geometry_configuration(road_network_configuration) {
  auto it = road_network_configuration.find(kStrRoadRuleBook);
  road_rule_book = it != road_network_configuration.end() ? std::make_optional(it->second) : std::nullopt;
  it = road_network_configuration.find(kStrTrafficLightBook);
  traffic_light_book = it != road_network_configuration.end() ? std::make_optional(it->second) : std::nullopt;
  it = road_network_configuration.find(kStrPhaseRingBook);
  phase_ring_book = it != road_network_configuration.end() ? std::make_optional(it->second) : std::nullopt;
  it = road_network_configuration.find(kStrIntersectionBook);
  intersection_book = it != road_network_configuration.end() ? std::make_optional(it->second) : std::nullopt;
}

std::map<std::string, std::string> RoadNetworkConfiguration::ToStringMap() const {
  auto rg_config = road_geometry_configuration.ToStringMap();
  if (road_rule_book.has_value()) {
    rg_config.emplace(kStrRoadRuleBook, road_rule_book.value());
  }
  if (traffic_light_book.has_value()) {
    rg_config.emplace(kStrTrafficLightBook, traffic_light_book.value());
  }
  if (phase_ring_book.has_value()) {
    rg_config.emplace(kStrPhaseRingBook, phase_ring_book.value());
  }
  if (intersection_book.has_value()) {
    rg_config.emplace(kStrIntersectionBook, intersection_book.value());
  }
  return rg_config;
}

}  // namespace builder
}  // namespace malidrive
