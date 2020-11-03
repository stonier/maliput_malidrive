##############################################################################
# Prep
##############################################################################

add_subdirectory(common)

##############################################################################
# Tests
##############################################################################

# Test binary name prefix, used by malipu_malidrive_build_tests
set(TEST_TYPE UNIT)


set(UNIT_TEST_SOURCES
  demo_test.cc
)

maliput_malidrive_build_tests(${UNIT_TEST_SOURCES})
