// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_IO_UTIL_HPP_
#define MUROSIM_IO_UTIL_HPP_

#include <vector>
#include <string>
#include <filesystem>

/// Small string/path helpers shared by the io module (world file parsing/naming).
namespace murosim::io::util
{

/// Splits string on a single separator character, dropping empty tokens.
std::vector<std::string> splitString(const std::string &string, const char separator);
/// Splits string on any character in separators, dropping empty tokens.
std::vector<std::string> splitString(const std::string &string, const std::string &separators);
/// @return the filename without its final extension (path::stem()).
std::string getFilename(const std::filesystem::path &path);

} // namespace murosim::io::util

#endif // MUROSIM_IO_UTIL_HPP_
