##############################################################################
# xodr
##############################################################################
add_library(xodr
  connection.cc
  db_manager.cc
  elevation_profile.cc
  geometry.cc
  header.cc
  junction.cc
  lane.cc
  lanes.cc
  lane_offset.cc
  lane_section.cc
  lane_width.cc
  lateral_profile.cc
  parser.cc
  road_header.cc
  road_link.cc
  road_type.cc
  tools.cc
  unit.cc
)

set_target_properties(xodr PROPERTIES OUTPUT_NAME xodr)

target_include_directories(
  xodr
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(xodr
  "maliput"
)

target_link_libraries(
  xodr
    common
    maliput::math
    tinyxml2
)

install(TARGETS xodr
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  FILES
    connection.h
    db_manager.h
    elevation_profile.h
    geometry.h
    header.h
    junction.h
    lanes.h
    lane_offset.h
    lane_section.h
    lane.h
    lane_link.h
    lane_width.h
    lateral_profile.h
    plan_view.h
    reference_geometry.h
    road_header.h
    road_link.h
    road_type.h
    unit.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/xodr
)
