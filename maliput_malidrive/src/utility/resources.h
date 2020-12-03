// Copyright 2018 Toyota Research Institute

#pragma once

#include <optional>
#include <string>

namespace utility {

// Finds a resource file by its `resource_name` name.
//
// The first valid file within path_directory/`resource_name` is returned.
// Where path_directory is indicated by MALIPUT_MALIDRIVE_RESOURCE_ROOT env variable.
// Path delimeter is assumed to be `/`.
// @param resource_name The name of the file.
// @return The complete file path to the resource.
// @throws std::runtime_error When the resource cannot be found under any
// of the possible combinations of 'path_directory/resource_name'.
std::string FindResource(const std::string& resource_name);

// Finds a resource file by its `resource_name` name.
//
// The first valid file within path_directory/`resource_name` is returned.
// Where path_directory is indicated by the path that the env variable `path_to_resources` holds.
// Path delimeter is assumed to be `/`.
// @param resource_name The name of the file.
// @param path_to_resources Env variable holding a directory path.
// @return The complete file path to the resource.
// @throws std::runtime_error When the resource cannot be found under any
// of the possible combinations of 'path_directory/resource_name'.
std::string FindResourceInPath(const std::string& resource_name, const std::string& path_to_resources);

}  // namespace utility
