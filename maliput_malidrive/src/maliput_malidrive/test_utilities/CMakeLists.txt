##############################################################################
# test_utilities
##############################################################################

add_library(test_utilities INTERFACE)

target_include_directories(
  test_utilities
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/test_utilities>
)

target_link_libraries(test_utilities
  INTERFACE
    common
    maliput::api
    maliput::math
    maliput::utilities
)

install(
  FILES
    function_stub.h
    ground_curve_stub.h
    xodr_testing_map_descriptions.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/test_utilities
)

# Install the exported target
install(TARGETS test_utilities EXPORT ${PROJECT_NAME}-targets)
