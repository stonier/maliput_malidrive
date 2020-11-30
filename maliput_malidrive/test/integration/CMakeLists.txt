##############################################################################
# Prep
##############################################################################

# Test binary name prefix, used by malidrive_build_integration_based_tests
set(TEST_TYPE INTEGRATION)

##############################################################################
# Tests
##############################################################################

set(INTEGRATION_TEST_SOURCES
  extensive_queries_test.cc
)

malidrive_build_integration_based_tests(${INTEGRATION_TEST_SOURCES})
