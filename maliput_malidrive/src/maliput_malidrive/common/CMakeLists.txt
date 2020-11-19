##############################################################################
# Library
##############################################################################

add_library(common
  common.cc
)
add_library(maliput_malidrive::common ALIAS common)
set_target_properties(common
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_common
)


target_include_directories(
  common
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(common
  PUBLIC
    "maliput"
)

target_link_libraries(common
  PUBLIC
    maliput::common
)

##############################################################################
# Installs
##############################################################################

install(TARGETS common
  EXPORT ${PROJECT_NAME}-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

install(
  FILES
    macros.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/common
)

ament_export_libraries(common)
