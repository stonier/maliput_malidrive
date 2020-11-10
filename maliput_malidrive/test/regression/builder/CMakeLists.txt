##############################################################################
# builder
##############################################################################

# Test binary name prefix, used by maliput_malidrive_build_tests
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(UNIT_BUILDER_TEST_SOURCES
  determine_tolerance_test.cc
  id_providers_test.cc
  malidrive_builder_tools_test.cc
  malidrive_road_geometry_builder_test.cc
  road_curve_factory_test.cc
  road_geometry_builder_base_test.cc
  rule_tools_test.cc
  simplify_geometries_test.cc
)

maliput_malidrive_build_tests(${UNIT_BUILDER_TEST_SOURCES})
