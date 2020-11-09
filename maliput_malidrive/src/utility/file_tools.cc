// Copyright 2020 Toyota Research Institute
#include "utility/file_tools.h"

namespace utility {

std::string GetFileNameFromPath(const std::string& file_path) {
  auto it = file_path.find_last_of('/');
  return it == std::string::npos ? file_path : file_path.substr(it + 1);
}

}  // namespace utility
