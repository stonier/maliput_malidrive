##############################################################################
# Tests
##############################################################################

##############################################################################
# Find Packages
##############################################################################

find_package(ament_cmake_gtest REQUIRED)

# Find the Python interpreter for running the check_test_ran.py script
find_package(PythonInterp QUIET)

##############################################################################
# Handle CFlags

include(${PROJECT_SOURCE_DIR}/cmake/MaliputMalidriveFlags.cmake)
##############################################################################

##############################################################################
# Macros
##############################################################################

macro(maliput_malidrive_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})

    # When integration tests are disabled, we just skip adding them.
    if(NOT MALIDRIVE_INTEGRATION_TESTS)
      if(${TEST_TYPE} MATCHES "INTEGRATION")
        message("Integration tests are disarmed, skipping: " ${BINARY_NAME})
        continue()
      endif()
    endif()

    ament_add_gtest(${BINARY_NAME} ${GTEST_SOURCE_file}
      TIMEOUT 240)

    target_include_directories(${BINARY_NAME}
      PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${PYTHON_INCLUDE_DIRS}
    )

    # Kind of an ugly catch-all bucket
    target_link_libraries(${BINARY_NAME}
        drake::drake
        fmt::fmt
        maliput::api
        maliput::base
        maliput::common
        maliput::geometry_base
        maliput::math
        maliput::test_utilities
        maliput_malidrive::base
        maliput_malidrive::builder
        maliput_malidrive::common
        maliput_malidrive::road_curve
        maliput_malidrive::test_utilities
        maliput_malidrive::xodr
        pthread
        utility
    )

    # Remove a warning in GTest.
    target_compile_options(${BINARY_NAME} PRIVATE "-Wno-sign-compare")

    if (PYTHONINTERP_FOUND)
      # Check that the test produced a result and create a failure if
      # it didn't. Guards against crashed and timed out tests.
      add_test(check_${BINARY_NAME} python
        ${PROJECT_SOURCE_DIR}/test/utils/check_test_ran.py
        ${CMAKE_BINARY_DIR}/test_results/maliput_malidrive/${BINARY_NAME}.gtest.xml)
    endif()
  endforeach()
endmacro()

##############################################################################
# Testing
##############################################################################

add_subdirectory(regression)
