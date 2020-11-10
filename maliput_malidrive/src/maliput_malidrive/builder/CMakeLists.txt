##############################################################################
# Malidrive builder
##############################################################################
add_library(builder
  id_providers.cc
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
    common
    maliput::api
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
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/builder
)
