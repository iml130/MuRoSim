// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_PLANNING_VORONOI_HPP_
#define MUROSIM_PLANNING_VORONOI_HPP_

#include <vector>
#include "planner.hpp"

namespace murosim::planning
{
   /// \warning NOT IMPLEMENTED. Constructing a Voronoi planner is allowed, but
   /// computePath() (and therefore plan()) throws std::runtime_error on every
   /// call. The type exists as a placeholder for a future implementation.
   class Voronoi : public Planner
   {
        public:
            /// @throws std::runtime_error always -- not implemented.
            virtual std::vector<Eigen::Vector2f> computePath(const murosim::grid::Map& map, const Eigen::Vector2f start, const Eigen::Vector2f goal) override;
            /// Returns a copy of this (unimplemented) planner.
            virtual std::unique_ptr<Planner> clone() const override;
            /// @return Planner::TYPE::VORONOI.
            virtual Planner::TYPE getType() const override;
        protected:

        private:
   };
} // namespace murosim::planning


#endif // MUROSIM_PLANNING_VORONOI_HPP_
