// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosim/collision/shape.hpp"

#include "murosim/collision/line.hpp"
#include "murosim/collision/circle.hpp"

namespace murosim::collision
{

bool shapeWithWorld(const murosim::geometry::Shape& shape, const Eigen::Affine2f &pose, const murosim::geometry::World &world)
{
    for (const murosim::geometry::Line &line : shape.getLines())
    {
        if(lineWithWorld(pose * line, world))
        {
            return true;
        }
    }

    for (const murosim::geometry::Circle &circle : shape.getCircles())
    {
        if (circleWithWorld(pose * circle, world))
        {
            return true;
        }
    }

    return false;
}

bool shapeWithShape(const murosim::geometry::Shape& shape1, const Eigen::Affine2f &pose1, const murosim::geometry::Shape &shape2, const Eigen::Affine2f &pose2)
{
    // transform shape1's primitives directly into shape2's frame: one relative
    // transform instead of transforming every shape2 primitive for every
    // shape1 primitive
    const Eigen::Affine2f relative = pose2.inverse() * pose1;

    for (const murosim::geometry::Line &line : shape1.getLines())
    {
        if (lineWithShape(relative * line, shape2))
        {
            return true;
        }
    }

    for (const murosim::geometry::Circle &circle : shape1.getCircles())
    {
        if (circleWithShape(relative * circle, shape2))
        {
            return true;
        }
    }

    return false;
}

} // namespace murosim::collision
