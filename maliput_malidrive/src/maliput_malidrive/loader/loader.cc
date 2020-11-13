// Copyright 2018 Toyota Research Institute
#include "maliput_malidrive/loader/loader.h"

#include <string>
#include <utility>

#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace loader {

template std::unique_ptr<maliput::api::RoadNetwork> Load<builder::RoadNetworkBuilder>(
    const builder::RoadNetworkConfiguration& road_network_configuration);

}  // namespace loader
}  // namespace malidrive
