// Copyright 2018 Toyota Research Institute
#include "utility/resources.h"

#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace utility {
namespace {

const char* kMalidriveEnvVariable{"MALIPUT_MALIDRIVE_RESOURCE_ROOT"};

// Retrieves a list of paths that live in `env_path` env variable.
std::vector<std::string> GetAllPathDirectories(const std::string& env_path) {
  std::istringstream path_stream(std::string(std::getenv(env_path.c_str())));
  const std::string delimeter{":"};
  std::vector<std::string> paths;
  std::string path;
  while (std::getline(path_stream, path, ':')) {
    paths.push_back(path);
  }
  return paths;
}

// Appends `relative_path` to the end of `base_path` adding the path delimiter
// ('/').
std::string AppendPath(const std::string& base_path, const std::string& relative_path) {
  const std::string path_delimeter("/");
  if (base_path.back() != *(path_delimeter.c_str())) return base_path + path_delimeter + relative_path;
  return base_path + relative_path;
}

}  // namespace

std::string FindResourceInPath(const std::string& resource_name, const std::string& path_to_resources) {
  const std::vector<std::string> env_paths = GetAllPathDirectories(path_to_resources);
  const std::string resources_folder{"resources"};
  for (const std::string& env_path : env_paths) {
    const std::string file_path = AppendPath(env_path, AppendPath(resources_folder, resource_name));
    if (std::ifstream(file_path)) {
      return file_path;
    }
  }
  throw std::runtime_error(std::string("Resource: ") + resource_name + std::string(" couldn't be found."));
}

std::string FindResource(const std::string& resource_name) {
  return FindResourceInPath(resource_name, kMalidriveEnvVariable);
}

}  // namespace utility
