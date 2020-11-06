##############################################################################
# Malidrive road_curve
##############################################################################

add_library(road_curve
  arc_ground_curve.cc
  lane_offset.cc
  line_ground_curve.cc
  malidrive_road_curve.cc
  open_range_validator.cc
  piecewise_function.cc
  piecewise_ground_curve.cc
  road_curve_offset.cc
)

set_target_properties(road_curve PROPERTIES OUTPUT_NAME road_curve)

target_include_directories(
  road_curve
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(road_curve
  "drake"
  "maliput"
)

target_link_libraries(
  road_curve
    common
    drake::drake
    maliput::api
    maliput::math
    maliput::utilities
)

install(TARGETS road_curve
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  FILES
    arc_ground_curve.h
    cubic_polynomial.h
    function.h
    ground_curve.h
    lane_offset.h
    line_ground_curve.h
    malidrive_road_curve.h
    open_range_validator.h
    piecewise_function.h
    piecewise_ground_curve.h
    road_curve_offset.h
    scaled_domain_function.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/road_curve
)
