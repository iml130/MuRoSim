// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/io/util.hpp"

namespace murosim::io::util
{

std::vector<std::string> splitString(const std::string &string, const char separator)
{
    // A single separator char is just the general case with a one-char set.
    return splitString(string, std::string(1, separator));
}

std::vector<std::string> splitString(const std::string &string, const std::string &separators)
{
    std::vector<std::string> split;

    for (size_t i = 0; i < string.size(); ++i) {
        size_t begin = i;

        while (i < string.size() && separators.find(string[i]) == std::string::npos) ++i;

        std::string substring(string.substr(begin, i - begin));

        if (!substring.empty()) {
            split.push_back(substring);
        }
    }

    return split;
}

std::string getFilename(const std::filesystem::path &path)
{
    // Filename without its final extension (equivalent to path::stem).
    return path.stem().string();
}

} // namespace murosim::io::util
