##############################################################################
# builder
##############################################################################

# Test binary name prefix, used by maliput_malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_BUILDER_TEST_SOURCES
  id_providers_test.cc
)

maliput_malidrive_build_tests(${UNIT_BUILDER_TEST_SOURCES})
