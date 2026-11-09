// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_MATH_CONSTANTS_HPP_
#define MUROSIM_MATH_CONSTANTS_HPP_

#include <numbers>

/// Single-precision angle constants used throughout the simulator. They are
/// the C++20 standard values (`std::numbers`), so no POSIX `M_PI` macros and
/// no feature-test defines are needed on any platform. The halves and
/// quarters are exact: dividing a float by a power of two never rounds.
namespace murosim::math
{
inline constexpr float PI = std::numbers::pi_v<float>;
inline constexpr float PI_2 = PI / 2.0f;
inline constexpr float PI_4 = PI / 4.0f;
inline constexpr float TWO_PI = 2.0f * PI;
} // namespace murosim::math

#endif // MUROSIM_MATH_CONSTANTS_HPP_
