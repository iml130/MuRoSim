// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosimrl/observation/observation_set.hpp"

#include "murosim/robot/robot.hpp"
#include "murosim/threading/thread_pool.hpp"

#include <memory>
#include <vector>

namespace murosimrlpy {

namespace {

size_t totalSize(const std::vector<size_t> &shape)
{
    if (shape.empty()) return 0;

    size_t total = 1;
    for (const size_t dim : shape)
    {
        total *= dim;
    }
    return total;
}

} // anonymous namespace

void initObservationSet(py::module_ &m)
{
    using murosimrl::observation::ObservationSet;
    using RobotList = std::vector<std::shared_ptr<murosim::robot::Robot>>;

    auto pyClass = py::class_<ObservationSet>(m, "ObservationSet",
        "Collection of robot observations computed in one thread-pool batch.\n"
        "Results are bit-identical to calling each observation serially.");

    pyClass.def(py::init<size_t>(), py::arg("min_chunk_size") = 16,
        "min_chunk_size: smallest robot count worth its own thread-pool job; "
        "small lists collapse into fewer chunks.");

    pyClass.def("add", &ObservationSet::add,
        "Append an observation; compute() returns arrays in add() order.",
        py::arg("observation"));

    pyClass.def("__len__", &ObservationSet::size,
        "Number of observations currently in the set.");

    pyClass.def("shapes", [](const ObservationSet &self, const RobotList &robots) {
        return self.shapes(robots);
    }, "One shape per operator, in add() order, for observing `robots`.",
       py::arg("robots"));

    // Allocate all numpy outputs under the GIL, then one GIL release and one
    // pool fan-out for every (observation x robot-chunk) job.
    pyClass.def("compute", [](const ObservationSet &self,
                              murosim::threading::ThreadPool &pool,
                              const RobotList &robots) {
        std::vector<py::array_t<float>> arrays;
        arrays.reserve(self.size());
        std::vector<float*> outputs(self.size(), nullptr);

        if (robots.empty())
        {
            for (size_t i = 0; i < self.size(); i++)
            {
                arrays.emplace_back();
            }
        }
        else
        {
            const std::vector<std::vector<size_t>> shapes = self.shapes(robots);

            for (size_t i = 0; i < shapes.size(); i++)
            {
                if (totalSize(shapes[i]) == 0)
                {
                    arrays.emplace_back();
                    continue;
                }

                arrays.emplace_back(shapes[i]);
                outputs[i] = arrays.back().mutable_data();
            }

            {
                py::gil_scoped_release gil;
                self.compute(pool, robots, outputs);
            }
        }

        py::list result;
        for (auto &array : arrays)
        {
            result.append(std::move(array));
        }
        return result;
    }, "Compute all observations for `robots` on the pool; returns a list of\n"
       "arrays in add() order.",
       py::arg("thread_pool"), py::arg("robots"));
}

} // namespace murosimrlpy
