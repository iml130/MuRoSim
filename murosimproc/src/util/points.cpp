// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "murosimproc/util/points.hpp"
#include "murosim/math/constants.hpp"

namespace murosimproc::util::points
{

void createRandomPointsRectangleUniform(std::vector<Eigen::Vector2f> &points, const murosim::random::RandomGenerator::result_type seed, const Eigen::Vector2f point1, const Eigen::Vector2f point2, const size_t number_of_points)
{
    murosim::random::RandomGenerator random_generator(seed);
    createRandomPointsRectangleUniform(points, random_generator, point1, point2, number_of_points);
}

void createRandomPointsRectangleUniform(std::vector<Eigen::Vector2f> &points, murosim::random::RandomGenerator &random_generator, const Eigen::Vector2f point1, const Eigen::Vector2f point2, const size_t number_of_points)
{
    const float min_x = std::min(point1.x(), point2.x());
    const float max_x = std::max(point1.x(), point2.x());
    const float min_y = std::min(point1.y(), point2.y());
    const float max_y = std::max(point1.y(), point2.y());

    assert(min_x < max_x);
    assert(min_y < max_y);

    std::uniform_real_distribution<float> distribution_x (min_x, max_x);
    std::uniform_real_distribution<float> distribution_y (min_y, max_y);

    for (size_t i = 0; i < number_of_points; ++i)
    {
        points.emplace_back(distribution_x(random_generator), distribution_y(random_generator));
    }
}

void createRandomPointsRectangleHexagon(std::vector<Eigen::Vector2f> &points, const murosim::random::RandomGenerator::result_type seed, const Eigen::Vector2f point1, const Eigen::Vector2f point2, const float spacing, const float noise)
{
    murosim::random::RandomGenerator random_generator(seed);
    createRandomPointsRectangleHexagon(points, random_generator, point1, point2, spacing, noise);
}

void createRandomPointsRectangleHexagon(std::vector<Eigen::Vector2f> &points, murosim::random::RandomGenerator &random_generator, const Eigen::Vector2f point1, const Eigen::Vector2f point2, const float spacing, const float noise)
{
    const float min_x = std::min(point1.x(), point2.x());
    const float max_x = std::max(point1.x(), point2.x());
    const float min_y = std::min(point1.y(), point2.y());
    const float max_y = std::max(point1.y(), point2.y());

    assert(min_x < max_x);
    assert(min_y < max_y);
    assert(spacing > 0.0f);
    assert(noise >= 0.0f);

    std::uniform_real_distribution<float> distribution_noise (-noise * 0.5f, noise * 0.5f);

    const float hex_height = spacing * std::sqrt(3.0f);
    size_t row = 0;
    for (float y = min_y + noise * 0.5f; y <= max_y; y += hex_height * 0.5f, ++row)
    {
        const float offset_x = (row % 2 == 0) ? 0.0f : spacing * 0.5f;
        for (float x = min_x + offset_x + noise * 0.5f; x <= max_x; x += spacing)
        {
            const float px = x + distribution_noise(random_generator);
            const float py = y + distribution_noise(random_generator);
            if (min_x <= px && px <= max_x && min_y <= py && py <= max_y)
            {
                points.emplace_back(px, py);
            }
        }
    }
}

void createRandomPointsRectanglePoissonDisk(std::vector<Eigen::Vector2f> &points, const murosim::random::RandomGenerator::result_type seed, const Eigen::Vector2f point1, const Eigen::Vector2f point2, const float min_distance, const float max_distance, const size_t k)
{
    murosim::random::RandomGenerator random_generator(seed);
    createRandomPointsRectanglePoissonDisk(points, random_generator, point1, point2, min_distance, max_distance, k);
}

void createRandomPointsRectanglePoissonDisk(std::vector<Eigen::Vector2f> &points, murosim::random::RandomGenerator &random_generator, const Eigen::Vector2f point1, const Eigen::Vector2f point2, const float min_distance, const float max_distance, const size_t k)
{
    const float min_x = std::min(point1.x(), point2.x());
    const float max_x = std::max(point1.x(), point2.x());
    const float min_y = std::min(point1.y(), point2.y());
    const float max_y = std::max(point1.y(), point2.y());

    assert(min_x < max_x);
    assert(min_y < max_y);
    assert(min_distance > 0.0f);
    assert(max_distance >= min_distance);
    assert(k > 0);

    const float width = max_x - min_x;
    const float height = max_y - min_y;

    // Grid cell size based on smallest possible radius for correct neighbor lookup
    const float cell_size = min_distance / std::sqrt(2.0f);
    const size_t grid_w = static_cast<size_t>(std::ceil(width / cell_size));
    const size_t grid_h = static_cast<size_t>(std::ceil(height / cell_size));

    // Neighborhood radius in grid cells — must cover max_distance
    const size_t neighborhood = static_cast<size_t>(std::ceil(max_distance / cell_size)) + 1;

    // Background grid (-1 = empty, otherwise index into points)
    std::vector<int> grid(grid_w * grid_h, -1);

    // Per-point radii (parallel to points vector)
    std::vector<float> radii;

    auto grid_index = [grid_w](size_t gx, size_t gy) -> size_t {
        return gy * grid_w + gx;
    };

    auto to_grid = [min_x, min_y, cell_size](const Eigen::Vector2f &p) -> std::pair<size_t, size_t> {
        return {
            static_cast<size_t>((p.x() - min_x) / cell_size),
            static_cast<size_t>((p.y() - min_y) / cell_size)
        };
    };

    std::uniform_real_distribution<float> dist_x(min_x, max_x);
    std::uniform_real_distribution<float> dist_y(min_y, max_y);
    std::uniform_real_distribution<float> dist_angle(0.0f, murosim::math::TWO_PI);
    std::uniform_real_distribution<float> dist_radius_assign(min_distance, max_distance);

    // Seed with one random point
    const Eigen::Vector2f first_point(dist_x(random_generator), dist_y(random_generator));
    const float first_radius = dist_radius_assign(random_generator);
    points.push_back(first_point);
    radii.push_back(first_radius);
    const auto [gx0, gy0] = to_grid(first_point);
    grid[grid_index(gx0, gy0)] = 0;

    std::vector<size_t> active_list;
    active_list.push_back(0);

    while (!active_list.empty())
    {
        // Pick a random active point
        std::uniform_int_distribution<size_t> dist_active(0, active_list.size() - 1);
        const size_t active_idx = dist_active(random_generator);
        const size_t active_point_idx = active_list[active_idx];
        const Eigen::Vector2f &active_point = points[active_point_idx];
        const float r_active = radii[active_point_idx];

        bool found = false;
        for (size_t i = 0; i < k; ++i)
        {
            // Assign candidate radius first so we know the minimum distance
            const float r_candidate = dist_radius_assign(random_generator);
            const float required_dist = std::max(r_active, r_candidate);

            // Generate candidate in annulus [required_dist, 2 * max_distance]
            std::uniform_real_distribution<float> dist_r(required_dist, 2.0f * max_distance);
            const float r = dist_r(random_generator);
            const float theta = dist_angle(random_generator);
            const Eigen::Vector2f candidate(
                active_point.x() + r * std::cos(theta),
                active_point.y() + r * std::sin(theta));

            // Bounds check
            if (candidate.x() < min_x || candidate.x() > max_x ||
                candidate.y() < min_y || candidate.y() > max_y)
            {
                continue;
            }

            // Grid cell of candidate
            const auto [cx, cy] = to_grid(candidate);

            // Check neighborhood for distance violations
            bool too_close = false;
            const size_t nx_min = (cx >= neighborhood) ? cx - neighborhood : 0;
            const size_t ny_min = (cy >= neighborhood) ? cy - neighborhood : 0;
            const size_t nx_max = std::min(cx + neighborhood, grid_w - 1);
            const size_t ny_max = std::min(cy + neighborhood, grid_h - 1);

            for (size_t ny = ny_min; ny <= ny_max && !too_close; ++ny)
            {
                for (size_t nx = nx_min; nx <= nx_max && !too_close; ++nx)
                {
                    const int idx = grid[grid_index(nx, ny)];
                    if (idx >= 0)
                    {
                        const float required_dist_sq =
                            std::max(r_candidate, radii[static_cast<size_t>(idx)]);
                        if ((points[static_cast<size_t>(idx)] - candidate).squaredNorm() <
                            required_dist_sq * required_dist_sq)
                        {
                            too_close = true;
                        }
                    }
                }
            }

            if (!too_close)
            {
                const size_t new_idx = points.size();
                points.push_back(candidate);
                radii.push_back(r_candidate);
                grid[grid_index(cx, cy)] = static_cast<int>(new_idx);
                active_list.push_back(new_idx);
                found = true;
                break;
            }
        }

        if (!found)
        {
            // Swap-remove from active list
            active_list[active_idx] = active_list.back();
            active_list.pop_back();
        }
    }
}

void choiceRandomPointsMinMax(std::vector<Eigen::Vector2f> &points, const murosim::random::RandomGenerator::result_type seed, const size_t min_number_of_points, const size_t max_number_of_points)
{
    murosim::random::RandomGenerator random_generator(seed);
    choiceRandomPointsMinMax(points, random_generator, min_number_of_points, max_number_of_points);
}

void choiceRandomPointsMinMax(std::vector<Eigen::Vector2f> &points, murosim::random::RandomGenerator &random_generator, const size_t min_number_of_points, const size_t max_number_of_points)
{
    assert(min_number_of_points <= max_number_of_points);
    assert(max_number_of_points > 0);

    std::uniform_int_distribution<size_t> distribution_number_of_points (min_number_of_points, max_number_of_points);
    const size_t number_of_points = distribution_number_of_points(random_generator);

    if (points.size() <= number_of_points)
    {
        return;
    }

    std::shuffle(points.begin(), points.end(), random_generator);
    points.resize(number_of_points);
}

} // namespace murosimproc::util::points
