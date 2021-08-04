#pragma once

#include <map>
#include <memory>
#include <string>

#include <maliput/api/road_network.h>

#include "maliput_malidrive/builder/road_network_configuration.h"

namespace malidrive {
namespace loader {

/// Builds a malidrive RoadNetwork.
///
/// Forwards a call to RoadNetworkBuilderT.
///
/// @param road_network_configuration A string-string map containing information about the RoadNetwork configuration
/// used during the loading process. The parameters that can be set are the following:
/// - 'road_geometry_id' : A string that works as ID of the RoadGeometry.
///                        Default: "maliput"
/// - 'opendrive_file' : Path to the XODR file to be loaded.
///                      Default: ""
/// - 'linear_tolerance' : RoadGeometry's linear tolerance.
///                        Default: "5e-2"
/// - 'angular_tolerance' : RoadGeometry's angular tolerance.
///                         Default: "1e-3"
/// - 'scale_length' : RoadGeometry's scale length.
///                    Default: "1.0"
/// - 'inertial_to_backend_frame_translation' : Translation from maliput to malidrive inertial frame.
///                                             The format of the 3-dimensional vector that is expected to be passed
///                                             should be {X , Y , Z}. Same format as maliput::math::Vector3 is
///                                             serialized. Default: "{0., 0., 0.}"
/// - 'build_policy' : Determines the use of concurrency when building the RoadGeometry.
///                    Options:
///                       1. "sequential"
///                       2. "parallel"
///                    Default: "sequential"
/// - 'simplification_policy' : Determines geometries simplification for the XODR's roads.
///                             Options:
///                               1. "none"
///                               2. "simplify"
///                             Default: "none"
/// - 'tolerance_selection_policy' : Tolerance selection method used by the builder.
///                                  Options:
///                                    1. "manual" : 'linear_tolerance' will be used.
///                                    2. "automatic" : 'linear_tolerance' is used as base and bigger tolerances are
///                                    used when the base tolerance make the builder to fail.
///                                  Default: "automatic"
/// - 'standard_strictness_policy' : Indicates how permissive builder should be with the XODR description.
///                                  Options:
///                                    1. "strict" : Do not permit any errors.
///                                    2. "allow_schema_errors" : Allow schema syntax errors.
///                                    3. "allow_semantic_errors" : Allow semantic errors.
///                                    4. "permissive": Allow all previous violations.
///                                  Default: "permissive"
/// - 'omit_nondrivable_lanes' : True for omitting building non-drivable lanes. False otherwise
///                              Options:
///                                1. "true", "True", "TRUE", "on", "On", "ON"
///                                2. "false", "False", "FALSE", "off", "Off", "OFF"
///                              Default: "true"
/// When parameters are omitted the default value will be used.
/// @return A unique_ptr to a RoadNetwork.
/// @tparam RoadNetworkBuilderT One of opendrive::builder::RoadNetworkBuilder or
///         builder::RoadNetworkBuilder.
template <class RoadNetworkBuilderT>
std::unique_ptr<maliput::api::RoadNetwork> Load(const std::map<std::string, std::string>& road_network_configuration) {
  return RoadNetworkBuilderT(road_network_configuration)();
}

/// TODO(#135): Deprecates this Load method.
///
/// Builds a malidrive RoadNetwork.
///
/// Forwards a call to RoadNetworkBuilderT.
///
/// @param road_network_configuration contains a RoadGeometryConfiguration and
///        the configuration files for RoadNetwork entity loaders. The
///        RoadGeometryConfiguration in the network configuration must be valid.
///
/// @throws std::runtime_err When the OpenDrive file cannot be loaded or
/// the RoadGeometry can not be constructed.
/// @return A unique_ptr to a RoadNetwork.
/// @tparam RoadNetworkBuilderT One of opendrive::builder::RoadNetworkBuilder or
///         builder::RoadNetworkBuilder.
template <class RoadNetworkBuilderT>
std::unique_ptr<maliput::api::RoadNetwork> Load(const builder::RoadNetworkConfiguration& road_network_configuration) {
  MALIDRIVE_VALIDATE(!road_network_configuration.road_geometry_configuration.opendrive_file.empty(), std::runtime_error,
                     "opendrive_file cannot be empty");
  return RoadNetworkBuilderT(road_network_configuration)();
}

}  // namespace loader
}  // namespace malidrive
