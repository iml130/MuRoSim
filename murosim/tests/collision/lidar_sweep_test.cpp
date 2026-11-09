// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "murosim/collision/lidar.hpp"
#include "murosim/robot/robot_builder.hpp"
#include "murosim/random/random_generator.hpp"
#include "murosim/math/constants.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace
{
using murosim::math::PI;
using murosim::geometry::Line;
using murosim::geometry::World;
using murosim::random::RandomGenerator;
using murosim::robot::Robot;
using murosim::simd::ArrayFloat;

constexpr float RANGE_MIN = 0.1f;
constexpr float RANGE_MAX = 10.0f;
constexpr float FREQUENCY = 10.0f; // scan duration 0.1 s
constexpr size_t NUM_RAYS = 360;
constexpr float WALL_X = 7.0f;
constexpr float WALL_DISTANCE = 2.0f;
constexpr uint64_t SEED = 42;
constexpr float INF = std::numeric_limits<float>::infinity();

// Marker written into the distance buffer to detect which rays a sweep touched
constexpr float SENTINEL = 42.0f;

// The ray closest to the perpendicular of a wall at distance d is at most half
// an angular step off it and reads at most d / cos(step / 2). For 1 deg steps
// and d <= 5 m that is at most 2e-4 m more.
constexpr float EPSILON_RAY = 1e-3f;

/// Vertical wall at x = WALL_X.
World wallWorld()
{
    return World({Line(WALL_X, -RANGE_MAX, WALL_X, RANGE_MAX)}, {});
}

std::shared_ptr<Robot> buildRobotWithLidar(const float fov, const size_t num_rays, const float frequency,
                                           const float angle_start = PI, const bool rotation_clockwise = false)
{
    murosim::robot::RobotBuilder builder;
    builder.setActuatorDifferentialSimple();
    builder.setShapeRobotRectangle(0.3f, 0.3f);
    builder.addLidar(Eigen::Affine2f::Identity(), fov, num_rays, RANGE_MIN, RANGE_MAX, frequency,
                     murosim::sensor::Lidar::NOISE_TYPE::NONE, 0.0f, angle_start, rotation_clockwise);
    return builder.build();
}

void fillWithSentinel(ArrayFloat &distance)
{
    for (size_t i = 0; i < distance.size(); i++)
    {
        distance[i] = SENTINEL;
    }
}

/// Shortest reading; +inf when no ray has a return.
float minDistance(const ArrayFloat &distance)
{
    float minimum = INF;
    for (size_t i = 0; i < distance.size(); i++)
    {
        minimum = std::min(minimum, distance[i]);
    }
    return minimum;
}

/// Expects that exactly the rays inside the (possibly wrapped) window
/// [begin, end) were updated, with one ray of quantization slack per boundary.
void assertSweptWindow(const ArrayFloat &distance, const size_t begin, const size_t end)
{
    const size_t num_rays = distance.size();

    const auto inside = [&](const size_t i)
    {
        return (begin <= end) ? (i >= begin && i < end) : (i >= begin || i < end);
    };
    const auto nearIndex = [&](const size_t i, const size_t boundary)
    {
        const size_t forward = (boundary + num_rays - i) % num_rays;
        return forward <= 1 || forward >= num_rays - 1;
    };

    for (size_t i = 0; i < num_rays; i++)
    {
        if (nearIndex(i, begin) || nearIndex(i, end))
        {
            continue; // quantization slack
        }

        if (inside(i))
        {
            ASSERT_NE(distance[i], SENTINEL) << "ray " << i << " should have been swept";
        }
        else
        {
            ASSERT_FLOAT_EQ(distance[i], SENTINEL) << "ray " << i << " swept outside the window";
        }
    }
}
} // namespace

// ============================================================================
// Test Fixture
// ============================================================================

// One robot with a full-circle lidar, WALL_DISTANCE in front of the wall
class CollisionLidarSweepTest : public ::testing::Test
{
protected:
    std::vector<std::shared_ptr<Robot>> robots{buildRobotWithLidar(2.0f * PI, NUM_RAYS, FREQUENCY)};
    World world = wallWorld();
    RandomGenerator random_generator{SEED};

    void SetUp() override
    {
        robots[0]->setPose(WALL_X - WALL_DISTANCE, 0.0f, 0.0f);
    }

    murosim::sensor::Lidar& lidar()
    {
        return robots[0]->getLidars()[0];
    }

    ArrayFloat& laserDistance()
    {
        return lidar().getArrayLaserDistanceBuffer();
    }
};

// ============================================================================
// Other Robots
// ============================================================================

// A robot built without setShapeLidar() is seen by other lidars as its body.
TEST(CollisionLidarSweepOtherRobots, RobotWithoutShapeLidarIsSeenAsItsBody)
{
    std::vector<std::shared_ptr<Robot>> robots{buildRobotWithLidar(2.0f * PI, NUM_RAYS, FREQUENCY),
                                               buildRobotWithLidar(2.0f * PI, NUM_RAYS, FREQUENCY)};
    robots[0]->setPose(0.0f, 0.0f, 0.0f);
    robots[1]->setPose(1.0f, 0.0f, 0.0f); // 0.3 m wide body: near face at x = 0.85
    RandomGenerator random_generator{SEED};

    murosim::collision::lidarSweepComplete(robots, wallWorld(), random_generator);

    EXPECT_NEAR(minDistance(robots[0]->getLidars()[0].getArrayLaserDistanceBuffer()), 0.85f, EPSILON_RAY);
}

// ============================================================================
// Coordinate Frame Tests
// ============================================================================

// Regression: the sweep functions used to pass world-frame geometry to the
// laser kernels, which assume the lidar sits at the origin, so every robot
// away from the origin got wrong readings in CONTINUOUS mode.
TEST_F(CollisionLidarSweepTest, SweepCompleteUsesLidarFrame)
{
    murosim::collision::lidarSweepComplete(robots, world, random_generator);

    // The closest ray hits the wall perpendicular at 2 m (a world-frame
    // computation would measure from the origin: 7 m)
    EXPECT_NEAR(minDistance(laserDistance()), WALL_DISTANCE, EPSILON_RAY);
}

// REP 117 end to end: a wall closer than range_min reads -inf (too close), a
// ray that sees nothing within range_max reads +inf (no return)
TEST_F(CollisionLidarSweepTest, SweepReportsTooCloseAndNoReturnAsInf)
{
    robots[0]->setPose(WALL_X - 0.5f * RANGE_MIN, 0.0f, 0.0f);

    murosim::collision::lidarSweepComplete(robots, world, random_generator);

    // ray 0 points at -pi, away from the wall; ray 180 at 0 deg, at the wall
    EXPECT_EQ(lidar().getLaserDistance(0), INF);
    EXPECT_EQ(lidar().getLaserDistance(180), -INF);
}

TEST_F(CollisionLidarSweepTest, SweepTimeFullRotationMatchesSynchronizedScan)
{
    // Full rotation: 0.1 s at 10 Hz (a fresh lidar starts at scan phase 0)
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.1f);

    std::vector<float> swept(NUM_RAYS);
    for (size_t i = 0; i < NUM_RAYS; i++)
    {
        swept[i] = laserDistance()[i];
    }

    // Reference: the SYNCHRONIZED-mode scan of the same scene (the wall ends
    // reach beyond RANGE_MAX, so the range limits must be applied as well)
    lidar().fillLaserDistanceWithInf();
    murosim::collision::lidarWithWorld(lidar(), robots[0]->getPose(), world);
    lidar().applyNoise(random_generator);

    for (size_t i = 0; i < NUM_RAYS; i++)
    {
        ASSERT_FLOAT_EQ(swept[i], laserDistance()[i]) << "ray " << i;
    }
}

// ============================================================================
// Partial Sweep Tests
// ============================================================================

// Regression: partial sweeps used to min-combine the current geometry into
// ALL rays (only the window was reset), so rays outside the window
// accumulated stale minima.
TEST_F(CollisionLidarSweepTest, SweepTimePartialWindowLeavesOtherRaysUntouched)
{
    fillWithSentinel(laserDistance());

    // Quarter rotation: rays [0, 90)
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.025f);

    for (size_t i = 0; i < NUM_RAYS; i++)
    {
        if (i < NUM_RAYS / 4)
        {
            ASSERT_NE(laserDistance()[i], SENTINEL) << "ray " << i << " not updated";
        }
        else
        {
            ASSERT_FLOAT_EQ(laserDistance()[i], SENTINEL) << "ray " << i << " modified outside window";
        }
    }
}

// Regression: an interval longer than one scan period was only recognized as
// a full sweep when its fmod-normalized endpoint indices happened to be equal.
TEST_F(CollisionLidarSweepTest, SweepTimeMultiRotationUpdatesAllRays)
{
    fillWithSentinel(laserDistance());

    // 2.5 rotations: more than one period always sweeps every ray
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.25f);

    for (size_t i = 0; i < NUM_RAYS; i++)
    {
        ASSERT_NE(laserDistance()[i], SENTINEL) << "ray " << i << " not updated";
    }
}

// Regression: each lidar keeps its own scan phase, so consecutive partial
// sweeps continue the rotation instead of restarting at ray 0, and the phase
// stays bounded in [0, scan_duration) (never a growing absolute time that would
// lose float precision over a long run).
TEST_F(CollisionLidarSweepTest, SweepTimeContinuesAcrossCalls)
{
    fillWithSentinel(laserDistance());
    EXPECT_FLOAT_EQ(lidar().getScanTime(), 0.0f);

    // First quarter rotation: rays [0, 90)
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.025f);
    EXPECT_FLOAT_EQ(lidar().getScanTime(), 0.025f);

    // Second quarter continues from phase 0.025: rays [90, 180), not [0, 90)
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.025f);
    EXPECT_FLOAT_EQ(lidar().getScanTime(), 0.05f); // bounded, not an absolute clock

    EXPECT_NE(laserDistance()[135], SENTINEL) << "second sweep did not continue the rotation";
    EXPECT_FLOAT_EQ(laserDistance()[270], SENTINEL) << "ray beyond both windows was touched";
}

// The published scan (getArrayLaserDistance) is the last complete rotation:
// during a partial sweep the work buffer already reflects the obstacle, but the
// published scan must stay frozen until the beam crosses the seam and a full
// rotation finishes.
TEST_F(CollisionLidarSweepTest, PublishedScanRefreshesOnlyWhenRotationCompletes)
{
    // Before any rotation completes the published scan is empty (+inf, no return)
    EXPECT_EQ(minDistance(lidar().getArrayLaserDistance()), INF);

    // Half a rotation: the work buffer already sees the wall, but no full
    // rotation finished, so the published scan must not change yet
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.05f);
    EXPECT_NEAR(minDistance(laserDistance()), WALL_DISTANCE, EPSILON_RAY) << "buffer should already see the wall";
    EXPECT_EQ(minDistance(lidar().getArrayLaserDistance()), INF) << "published scan changed mid-rotation";

    // The second half crosses the seam and completes the rotation: publish
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.05f);
    EXPECT_NEAR(minDistance(lidar().getArrayLaserDistance()), WALL_DISTANCE, EPSILON_RAY)
        << "published scan not refreshed after a full rotation";
}

// ============================================================================
// Dead Time Tests
// ============================================================================

// With fov < 2*pi the beam spends part of each rotation outside the field of
// view, during which no ray may update. A half-circle lidar at 10 Hz has a
// rotation period of 0.1 s but measures only during the first 0.05 s of it.
TEST(CollisionLidarSweepDeadTime, DeadZoneWindowLeavesAllRaysUntouched)
{
    const World world = wallWorld();
    RandomGenerator random_generator(SEED);

    std::vector<std::shared_ptr<Robot>> robots = {buildRobotWithLidar(PI, 180, FREQUENCY)};
    robots[0]->setPose(5.0f, 0.0f, 0.0f);

    ArrayFloat &distance = robots[0]->getLidars()[0].getArrayLaserDistanceBuffer();
    fillWithSentinel(distance);

    // Position the beam at phase 0.06 (already past the 0.05 FOV edge), then
    // sweep 0.03 s: the beam is outside the FOV for the whole interval
    robots[0]->getLidars()[0].advanceScanTime(0.06f);
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.03f);

    for (size_t i = 0; i < distance.size(); i++)
    {
        ASSERT_FLOAT_EQ(distance[i], SENTINEL) << "ray " << i << " updated during dead time";
    }
}

TEST(CollisionLidarSweepDeadTime, WindowEndingInDeadZoneStopsAtFovEdge)
{
    const World world = wallWorld();
    RandomGenerator random_generator(SEED);

    std::vector<std::shared_ptr<Robot>> robots = {buildRobotWithLidar(PI, 180, FREQUENCY)};
    robots[0]->setPose(5.0f, 0.0f, 0.0f);

    ArrayFloat &distance = robots[0]->getLidars()[0].getArrayLaserDistanceBuffer();
    fillWithSentinel(distance);

    // From phase 0.04, sweep 0.03 s: measuring until 0.05 (rays [144, 180)),
    // then dead time
    robots[0]->getLidars()[0].advanceScanTime(0.04f);
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.03f);

    assertSweptWindow(distance, 144, 180);
}

TEST(CollisionLidarSweepDeadTime, WindowAcrossDeadZoneWrapsIntoNextScan)
{
    const World world = wallWorld();
    RandomGenerator random_generator(SEED);

    std::vector<std::shared_ptr<Robot>> robots = {buildRobotWithLidar(PI, 180, FREQUENCY)};
    robots[0]->setPose(5.0f, 0.0f, 0.0f);

    ArrayFloat &distance = robots[0]->getLidars()[0].getArrayLaserDistanceBuffer();
    fillWithSentinel(distance);

    // From phase 0.04, sweep 0.07 s: tail of this FOV (rays [144, 180)), the
    // dead zone, and the start of the next rotation (rays [0, 36))
    robots[0]->getLidars()[0].advanceScanTime(0.04f);
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.07f);

    assertSweptWindow(distance, 144, 36);
}

// Regression: a FOV within half a full-circle ray step of 2*pi is laid out by
// Lidar::createLaser as a seamless full circle with no dead zone.
// Lidar::indexWindowForTimeInterval must use that same predicate; otherwise the
// trailing ray of each rotation falls into a phantom dead zone and never
// updates. Here fov = 5.5 with 4 rays at 10 Hz is a full-circle layout
// (2*pi - 5.5 = 0.783 < (2*pi / 4) / 2 = 0.785), so the four rays span the whole
// rotation: ray 3 owns the time slot [0.075, 0.1). The naive `fov < 2*pi` test
// would instead end the measuring window at 0.0875 s, mapping phase 0.088 into
// dead time and dropping ray 3.
TEST(CollisionLidarSweepDeadTime, NearFullCircleFovHasNoPhantomDeadZone)
{
    const World world = wallWorld();
    RandomGenerator random_generator(SEED);

    std::vector<std::shared_ptr<Robot>> robots = {buildRobotWithLidar(5.5f, 4, FREQUENCY)};
    robots[0]->setPose(5.0f, 0.0f, 0.0f);

    ArrayFloat &distance = robots[0]->getLidars()[0].getArrayLaserDistanceBuffer();
    fillWithSentinel(distance);

    // From phase 0.088, sweep 0.013 s: inside ray 3's slot, crossing the
    // rotation boundary. With the correct full-circle mapping this sweeps ray 3;
    // the old dead-zone math (measuring window ending at 0.0875) returned an
    // empty window and left it.
    robots[0]->getLidars()[0].advanceScanTime(0.088f);
    murosim::collision::lidarSweepTime(robots, world, random_generator, 0.013f);

    EXPECT_NE(distance[3], SENTINEL) << "trailing ray dropped into a phantom dead zone";
    // The earlier rays are outside this window and must stay untouched
    EXPECT_FLOAT_EQ(distance[0], SENTINEL);
    EXPECT_FLOAT_EQ(distance[1], SENTINEL);
    EXPECT_FLOAT_EQ(distance[2], SENTINEL);
}

// ============================================================================
// Scan Direction Tests
// ============================================================================

// With a wall only on the +y side, the second quarter of a full rotation sees
// the wall for a clockwise scanner (+90 deg -> 0) but not for a
// counter-clockwise one (-90 deg -> 0).
TEST(CollisionLidarSweepDirection, ClockwiseSweepCoversPhysicallySweptAngles)
{
    const float wall_y = 5.0f;
    const World world({Line(-RANGE_MAX, wall_y, RANGE_MAX, wall_y)}, {});

    // Rays 90 and 179 are skipped below as quantisation slack, so the closest
    // checked ray (91, at 89 deg clockwise) is one step off the perpendicular:
    // 5 m / cos(1 deg) - 5 m = 7.6e-4 m
    constexpr float EPSILON_WINDOW = 2e-3f;

    for (const bool clockwise : {false, true})
    {
        RandomGenerator random_generator(SEED);

        std::vector<std::shared_ptr<Robot>> robots =
            {buildRobotWithLidar(2.0f * PI, NUM_RAYS, FREQUENCY, PI, clockwise)};
        robots[0]->setPose(0.0f, 0.0f, 0.0f);

        ArrayFloat &distance = robots[0]->getLidars()[0].getArrayLaserDistanceBuffer();
        fillWithSentinel(distance);

        // Second quarter of the rotation: from phase 0.025, sweep 0.025 s (rays [90, 180))
        robots[0]->getLidars()[0].advanceScanTime(0.025f);
        murosim::collision::lidarSweepTime(robots, world, random_generator, 0.025f);

        float minimum = INF;
        for (size_t i = 91; i < 179; i++)
        {
            ASSERT_NE(distance[i], SENTINEL) << "ray " << i << " not swept";
            minimum = std::min(minimum, distance[i]);
        }

        if (clockwise)
        {
            // The beam swept from +90 deg toward 0 and saw the wall
            EXPECT_NEAR(minimum, wall_y, EPSILON_WINDOW);
        }
        else
        {
            // The beam swept from -90 deg toward 0: no wall on that side,
            // every ray reads +inf (no return)
            EXPECT_EQ(minimum, INF);
        }
    }
}
