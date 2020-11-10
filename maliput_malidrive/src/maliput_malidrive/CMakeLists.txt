##############################################################################
# Malidrive
##############################################################################

##############################################################################
# Handle CFlags

include(${PROJECT_SOURCE_DIR}/cmake/MaliputMalidriveFlags.cmake)
##############################################################################

add_subdirectory(common)
add_subdirectory(road_curve)
add_subdirectory(test_utilities)
add_subdirectory(xodr)

install(
  FILES
    constants.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
