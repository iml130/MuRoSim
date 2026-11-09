// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosimrl/observation/robot_simulation/pose_normalized.hpp"
#include "fixtures.hpp"
#include "murosim/math/constants.hpp"
#include <memory>
#include <vector>

namespace
{
using murosim::math::PI_2;
using murosim::simulation::Simulation;
using murosimrl::observation::robot_simulation::PoseNormalized;
using fixtures::EPSILON;

constexpr float NO_NORMALIZATION = 0.0f;
constexpr float NORMALIZE_DISTANCE = 10.0f;
constexpr bool AS_ANGLE = false;
constexpr bool AS_UNIT_VECTOR = true;
constexpr size_t NO_STACK = 0;
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

class PoseNormalizedTest : public fixtures::SingleRobotTest
{
protected:
    void SetUp() override
    {
        fixtures::SingleRobotTest::SetUp();
        robot->setPose(3.0f, 4.0f, PI_2);
        step();
    }
};

// ============================================================================
// Angle Tests
// ============================================================================

TEST_F(PoseNormalizedTest, AngleShapeIsRobotsByThree)
{
    const PoseNormalized observation(NO_NORMALIZATION, AS_ANGLE, NO_STACK);

    EXPECT_EQ(observation.shape(sims()), (std::vector<size_t>{1, 3}));
    EXPECT_EQ(observation.name(), "pose_normalized");
    EXPECT_FALSE(observation.orientationAsUnitVector());
}

TEST_F(PoseNormalizedTest, RawOffsetAndAngle)
{
    const PoseNormalized observation(NO_NORMALIZATION, AS_ANGLE, NO_STACK);

    const std::vector<float> output = observation(sims());

    ASSERT_EQ(output.size(), static_cast<size_t>(3));
    EXPECT_FLOAT_EQ(output[0], 3.0f);
    EXPECT_FLOAT_EQ(output[1], 4.0f);
    EXPECT_NEAR(output[2], PI_2, EPSILON);
}

TEST_F(PoseNormalizedTest, NormalizedOffsetIsShiftedByHalf)
{
    const PoseNormalized observation(NORMALIZE_DISTANCE, AS_ANGLE, NO_STACK);

    const std::vector<float> output = observation(sims());

    EXPECT_NEAR(output[0], 0.5f + 3.0f / NORMALIZE_DISTANCE, EPSILON);
    EXPECT_NEAR(output[1], 0.5f + 4.0f / NORMALIZE_DISTANCE, EPSILON);
    EXPECT_NEAR(output[2], PI_2, EPSILON); // the angle is never normalized
    EXPECT_FLOAT_EQ(observation.normalizeDistance(), NORMALIZE_DISTANCE);
}

// ============================================================================
// Unit Vector Tests
// ============================================================================

TEST_F(PoseNormalizedTest, UnitVectorShapeIsRobotsByFour)
{
    const PoseNormalized observation(NO_NORMALIZATION, AS_UNIT_VECTOR, NO_STACK);

    EXPECT_EQ(observation.shape(sims()), (std::vector<size_t>{1, 4}));
    EXPECT_TRUE(observation.orientationAsUnitVector());
}

TEST_F(PoseNormalizedTest, UnitVectorValues)
{
    const PoseNormalized observation(NORMALIZE_DISTANCE, AS_UNIT_VECTOR, NO_STACK);

    const std::vector<float> output = observation(sims());

    ASSERT_EQ(output.size(), static_cast<size_t>(4));
    EXPECT_NEAR(output[0], 0.5f + 3.0f / NORMALIZE_DISTANCE, EPSILON);
    EXPECT_NEAR(output[1], 0.5f + 4.0f / NORMALIZE_DISTANCE, EPSILON);
    EXPECT_NEAR(output[2], 0.0f, EPSILON); // cos(pi/2)
    EXPECT_NEAR(output[3], 1.0f, EPSILON); // sin(pi/2)
}

// ============================================================================
// Batch Tests
// ============================================================================

TEST_F(PoseNormalizedTest, BatchCoversAllRobotsOfAllSimulations)
{
    const std::shared_ptr<Simulation> second = fixtures::makeSimulation("second");
    const std::shared_ptr<murosim::robot::Robot> other = fixtures::holonomicRobot("other");
    second->addRobot(other);
    second->reset();
    other->setPose(-1.0f, -2.0f, 0.0f);
    second->simulate();
    const PoseNormalized observation(NO_NORMALIZATION, AS_ANGLE, NO_STACK);

    const std::vector<float> output = observation(fixtures::SimulationList{sim, second});

    ASSERT_EQ(output.size(), static_cast<size_t>(6));
    EXPECT_FLOAT_EQ(output[0], 3.0f);
    EXPECT_FLOAT_EQ(output[3], -1.0f);
    EXPECT_FLOAT_EQ(output[4], -2.0f);
}

TEST_F(PoseNormalizedTest, StackIsNewestFirst)
{
    robot->setPose(1.0f, 1.0f, 0.0f);
    step();
    const PoseNormalized observation(NO_NORMALIZATION, AS_ANGLE, 2);

    const std::vector<float> output = observation(sims());

    ASSERT_EQ(observation.shape(sims()), (std::vector<size_t>{1, 2, 3}));
    EXPECT_FLOAT_EQ(output[0], 1.0f); // newest
    EXPECT_FLOAT_EQ(output[3], 3.0f); // one step back
    EXPECT_NEAR(output[5], PI_2, EPSILON);
}
