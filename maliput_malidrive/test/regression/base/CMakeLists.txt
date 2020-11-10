##############################################################################
# base
##############################################################################

# Test binary name prefix, used by maliput_malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_BASE_TEST_SOURCES
  malidrive_lane_test.cc
  malidrive_segment_test.cc
)

maliput_malidrive_build_tests(${UNIT_BASE_TEST_SOURCES})
