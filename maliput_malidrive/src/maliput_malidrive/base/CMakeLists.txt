##############################################################################
# MaliputMalidrive base
##############################################################################
add_library(base
  malidrive_lane.cc
  malidrive_road_geometry.cc
  world_to_opendrive_transform.cc
)

set_target_properties(base PROPERTIES OUTPUT_NAME base)

target_include_directories(
  base
  PUBLIC
    ${PROJ4_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(base
  "drake"
  "maliput"
)

target_link_libraries(
  base
    builder
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

install(TARGETS base
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  FILES
    inertial_to_lane_mapping_config.h
    malidrive_lane.h
    malidrive_road_geometry.h
    malidrive_segment.h
    world_to_opendrive_transform.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/base/
)
