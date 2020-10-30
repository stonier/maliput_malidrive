##############################################################################
# Configured
##############################################################################

configure_file(version.h.in
  ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/version.h
)

##############################################################################
# Installs
##############################################################################

# Files
install(
  FILES
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/version.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
