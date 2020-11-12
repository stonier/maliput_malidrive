include(${PROJECT_SOURCE_DIR}/cmake/MaliputMalidriveFlags.cmake)

install(
  FILES
    log_level_flag.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/applications
)

add_executable(xodr_query xodr_query.cc)
target_link_libraries(
  xodr_query
  PUBLIC
    gflags
    maliput::common
    utility
    xodr
)
install(
  TARGETS
     xodr_query
  DESTINATION
    ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}/applications
)

add_executable(xodr_validate xodr_validate.cc)
target_link_libraries(
  xodr_validate
  PUBLIC
    gflags
    maliput::common
    utility
    xodr
)
install(
  TARGETS
     xodr_validate
  DESTINATION
    ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}/applications
)
