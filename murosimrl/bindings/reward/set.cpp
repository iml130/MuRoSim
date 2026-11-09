// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosimrl/reward/reward_set.hpp"

#include "murosim/robot/robot.hpp"
#include "murosim/threading/thread_pool.hpp"

#include <memory>
#include <vector>

namespace murosimrlpy {

void initRewardSet(py::module_ &m)
{
    using murosimrl::reward::RewardSet;
    using RobotList = std::vector<std::shared_ptr<murosim::robot::Robot>>;

    auto pyClass = py::class_<RewardSet>(m, "RewardSet",
        "Collection of robot rewards summed in one thread-pool batch.\n"
        "The sum runs in add() order per robot, so the result is\n"
        "bit-identical to summing the rewards serially.");

    pyClass.def(py::init<size_t>(), py::arg("min_chunk_size") = 16,
        "min_chunk_size: smallest robot count worth its own thread-pool job; "
        "small lists collapse into fewer chunks.");

    pyClass.def("add", &RewardSet::add,
        "Append a reward; compute() sums all rewards in add() order.",
        py::arg("reward"));

    pyClass.def("__len__", &RewardSet::size,
        "Number of rewards currently in the set.");

    pyClass.def("compute", [](const RewardSet &self,
                              murosim::threading::ThreadPool &pool,
                              const RobotList &robots) {
        if (robots.empty())
        {
            return py::array_t<float>();
        }

        py::array_t<float> array(self.shape(robots));
        float* data = array.mutable_data();

        {
            py::gil_scoped_release gil;
            self.compute(pool, robots, data);
        }

        return array;
    }, "Sum all rewards for `robots` on the pool; returns a (N, 1) array.",
       py::arg("thread_pool"), py::arg("robots"));
}

} // namespace murosimrlpy
