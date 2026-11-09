// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIMPROC_GENERATORS_RANDOM_SPARSE_GENERATOR_HPP_
#define MUROSIMPROC_GENERATORS_RANDOM_SPARSE_GENERATOR_HPP_

#include "murosim/io/data_container.hpp"
#include "murosimproc/generators/generator.hpp"
#include "murosimproc/obstacle/shape_distribution.hpp"

namespace murosimproc::generators
{

namespace defaults {
    constexpr float MIN_OBSTACLE_SCALE = 0.3f;
    constexpr float MAX_OBSTACLE_SCALE = 2.0f;
    constexpr float PLACEMENT_DENSITY_RATE = 1.0f;

    // ShapeDistribution{circle, polygon, decompose}; min_polygon_radius keeps its default
    inline const obstacle::ShapeDistribution SHAPE_DISTRIBUTION{
        RandomRange(0.5f),
        RandomRange(0.5f),
        RandomRange(0.3f),
    };
}

class RandomSparseGenerator: public murosimproc::generators::Generator
{
private:
    RandomRange obstacle_size_;
    obstacle::ShapeDistribution shape_distribution_;
    RandomRange placement_density_rate_;

    void validateRandomSparseConfig() const;

    /**
     * @brief Calculate max placement attempts based on world size and density rate
     * @param world_size Size of the world
     * @return Number of placement attempts to use
     */
    size_t calculateMaxPlacementAttempts(float world_size, float placement_density_rate) const;

protected:
    void generate(murosim::random::RandomGenerator::result_type seed, std::vector<murosim::geometry::Line> &lines, std::vector<murosim::geometry::Circle> &circles, std::vector<murosim::navigation::Node> &nodes) const override;

public:
    /**
     * @brief Convert a shape entropy value to a ShapeDistribution.
     */
    static obstacle::ShapeDistribution entropyToShapeDistribution(const RandomRange& entropy);

    /**
     * @brief Primary constructor with explicit shape distribution.
     */
    RandomSparseGenerator(
        size_t min_node_count,
        float node_radius,
        RandomRange world_size,
        RandomRange spacing,
        RandomRange obstacle_size,
        RandomRange placement_density_rate,
        obstacle::ShapeDistribution shape_distribution);

    /**
     * @brief Convenience constructor with shape entropy (converts to ShapeDistribution).
     */
    RandomSparseGenerator(
        size_t min_node_count,
        float node_radius,
        RandomRange world_size,
        RandomRange spacing,
        RandomRange obstacle_size,
        RandomRange placement_density_rate,
        RandomRange shape_entropy = 0.5f);

    const RandomRange& getObstacleSize() const;
    const obstacle::ShapeDistribution& getShapeDistribution() const;
    const RandomRange& getPlacementDensityRate() const;
};

} // namespace murosimproc::generators


#endif // MUROSIMPROC_GENERATORS_RANDOM_SPARSE_GENERATOR_HPP_
