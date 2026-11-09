// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_MODELS_WORLD_HPP_
#define MUROSIM_MODELS_WORLD_HPP_

#include "murosim/io/data_container.hpp"

/// Factory functions producing small preset test worlds as ready-to-use
/// DataContainer instances (world outline plus obstacle geometry). Every
/// function takes the same two parameters: `world_size`, the overall extent
/// of the world (metres, must be positive), and `space`, the corridor/gap
/// width (metres, must be positive and not exceed world_size).
namespace murosim::models::world
{

/// Straight single corridor spanning the world.
murosim::io::DataContainer line(const float world_size=10.0f, const float space=1.0f);
/// Empty world with only the outer boundary, no internal obstacles.
murosim::io::DataContainer empty(const float world_size=10.0f, const float space=1.0f);
/// Four-way crossing of two perpendicular corridors.
murosim::io::DataContainer intersection(const float world_size=10.0f, const float space=1.0f);
/// Circular roundabout connecting corridors from multiple directions.
murosim::io::DataContainer roundabout(const float world_size=10.0f, const float space=1.0f);
/// Single corridor bending through a curve.
murosim::io::DataContainer curve(const float world_size=10.0f, const float space=1.0f);
/// Dumbbell-shaped layout: two open areas joined by a corridor.
murosim::io::DataContainer bone(const float world_size=10.0f, const float space=1.0f);

} // namespace murosim::models::world

#endif // MUROSIM_MODELS_WORLD_HPP_
