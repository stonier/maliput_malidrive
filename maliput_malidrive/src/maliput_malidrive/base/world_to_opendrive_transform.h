// Copyright 2019 Toyota Research Institute.  All rights reserved.
#pragma once

#include "maliput/api/lane_data.h"  // Defines GeoPosition

#include "maliput_malidrive/common/macros.h"

namespace malidrive {

/// The origin of the maliput world frame may not be coincident with the origin of the OpenDrive file's inertial frame.
/// This class defines the transformation between Maliput's world frame (W) and the OpenDrive file's inertial frame (O).
/// The frames are defined to have a common orientation and scale, only translations are supported.
// TODO(andrew.best) We may want to add support for orientation offsets at some point.
class WorldToOpenDriveTransform {
 public:
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(WorldToOpenDriveTransform);

  WorldToOpenDriveTransform() = delete;

  /// The input parameters specify the location of the origin of the OpenDrive inertial frame in the maliput
  /// world frame.
  WorldToOpenDriveTransform(double x_offset, double y_offset, double z_offset);

  /// Creates an identity WorldToOpenDriveTransform.
  static WorldToOpenDriveTransform Identity();

  /// Converts @p position_world to an equivalent position in the OpenDrive inertial frame.
  maliput::api::GeoPosition WorldToOpenDrive(const maliput::api::GeoPosition& position_W) const;

  /// Converts world coordinates (@p x, @p y, @p z) to be an equivalent position in the OpenDrive inertial frame.
  maliput::api::GeoPosition WorldToOpenDrive(double x, double y, double z) const;

  /// Converts @p position_opendrive to be an equivalent position in the world frame.
  maliput::api::GeoPosition OpenDriveToWorld(const maliput::api::GeoPosition& position_O) const;

 private:
  // Position of the OpenDrive origin(O) in World frame(W).
  maliput::api::GeoPosition pos_OW_;
};

}  // namespace malidrive
