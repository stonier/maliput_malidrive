// Copyright 2020 Toyota Research Institute
#pragma once

#include <string>

namespace utility {

/// Finds the last occurrence of '/' in `file_path` and returns the substring
/// from it. When no '/' is found, `file_path` is returned.
/// @param file_path Is the file path of the file.
/// @returns The name of the file pointing by `file_path`.
std::string GetFileNameFromPath(const std::string& file_path);

}  // namespace utility
