// Copyright 2019 Toyota Research Institute.  All rights reserved.
#include "maliput_malidrive/base/world_to_opendrive_transform.h"

using maliput::api::GeoPosition;

namespace malidrive {

WorldToOpenDriveTransform::WorldToOpenDriveTransform(double x_offset, double y_offset, double z_offset)
    : pos_OW_(x_offset, y_offset, z_offset) {}

WorldToOpenDriveTransform WorldToOpenDriveTransform::Identity() { return WorldToOpenDriveTransform(0, 0, 0); }

GeoPosition WorldToOpenDriveTransform::WorldToOpenDrive(const GeoPosition& position_W) const {
  return position_W - pos_OW_;
}

GeoPosition WorldToOpenDriveTransform::WorldToOpenDrive(double x, double y, double z) const {
  return WorldToOpenDrive(GeoPosition(x, y, z));
}

GeoPosition WorldToOpenDriveTransform::OpenDriveToWorld(const GeoPosition& position_O) const {
  return position_O + pos_OW_;
}

}  // namespace malidrive.
