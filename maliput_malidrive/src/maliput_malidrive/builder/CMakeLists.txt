##############################################################################
# Malidrive builder
##############################################################################
add_library(builder
  determine_tolerance.cc
  id_providers.cc
  malidrive_builder_tools.cc
  malidrive_direction_usage_builder.cc
  malidrive_discrete_value_rule_state_provider_builder.cc
  malidrive_range_value_rule_state_provider_builder.cc
  malidrive_road_geometry_builder.cc
  malidrive_road_network_builder.cc
  malidrive_rule_registry_builder.cc
  malidrive_road_rulebook_builder.cc
  malidrive_speed_limit_builder.cc
  road_curve_factory.cc
  road_geometry_builder_base.cc
  rule_tools.cc
  simplify_geometries.cc
)

set_target_properties(builder PROPERTIES OUTPUT_NAME builder)

target_include_directories(
  builder
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(builder
  "maliput"
)

target_link_libraries(
  builder
    base
    common
    drake::drake
    maliput::api
    maliput::base
    maliput::geometry_base
    maliput::math
    maliput::utilities
    road_curve
    xodr
)

install(TARGETS builder
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  FILES
    id_providers.h
    malidrive_builder_tools.h
    malidrive_road_network_builder.h
    malidrive_road_geometry_builder.h
    road_curve_factory.h
    road_geometry_builder_base.h
    road_geometry_configuration.h
    road_network_configuration.h
    road_network_builder_base.h
    rule_tools.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/builder
)
