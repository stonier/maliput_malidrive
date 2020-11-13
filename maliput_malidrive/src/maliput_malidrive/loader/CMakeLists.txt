##############################################################################
# Malidrive loader
##############################################################################
add_library(loader
  loader.cc
)

set_target_properties(loader PROPERTIES OUTPUT_NAME loader)

target_include_directories(
  loader
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(loader
  "maliput"
)

target_link_libraries(
  loader
    builder
    common
    maliput::api
    maliput::base
    maliput::geometry_base
    maliput::math
    maliput::utilities
)

install(TARGETS loader
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  FILES
    loader.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/loader
)
