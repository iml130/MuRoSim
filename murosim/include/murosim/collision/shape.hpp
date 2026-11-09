// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_COLLISION_SHAPE_HPP_
#define MUROSIM_COLLISION_SHAPE_HPP_

#include "murosim/geometry/shape.hpp"
#include "murosim/geometry/world.hpp"

namespace murosim::collision
{
    /// Tests shape, posed at pose in the world, against every line and circle in world.
    /// @return true on any overlap of any of shape's primitives.
    bool shapeWithWorld(const murosim::geometry::Shape& shape, const Eigen::Affine2f &pose, const murosim::geometry::World &world);
    /// Tests shape1 (posed at pose1) against shape2 (posed at pose2) for overlap.
    /// Transforms shape1's primitives directly into shape2's local frame via one
    /// relative transform, rather than transforming every shape2 primitive per shape1 primitive.
    bool shapeWithShape(const murosim::geometry::Shape& shape1, const Eigen::Affine2f &pose1, const murosim::geometry::Shape &shape2, const Eigen::Affine2f &pose2);
}

#endif // MUROSIM_COLLISION_SHAPE_HPP_
