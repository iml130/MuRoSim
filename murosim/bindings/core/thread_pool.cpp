// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "../common.hpp"

#include "murosim/simulation/simulation.hpp"
#include "murosim/threading/thread_pool.hpp"

namespace murosimpy {

void initCoreThreadPool(pybind11::module &m)
{
    auto pyThreadPool = pybind11::class_<murosim::threading::ThreadPool>(m, "ThreadPool",
        R"doc(Fixed-size pool of worker threads for stepping many simulations in parallel.

Each execute_* method blocks the calling thread until every job it queued
has run (or one has thrown, in which case the exception is re-raised after
all jobs finish). The GIL is released while jobs run, so multiple Python
threads calling into the pool do not serialize on it. Not copyable; the
pool's C++ destructor stops and joins all worker threads.)doc");

    pyThreadPool.def(pybind11::init<const size_t>(),
        "Create a thread pool with specified number of threads.\n\n"
        "Args:\n"
        "    number_of_threads: Number of worker threads (0 = auto-detect)",
        pybind11::arg("number_of_threads") = static_cast<size_t>(0));

    pyThreadPool.def_property_readonly("thread_count", &murosim::threading::ThreadPool::numberOfThreads,
        "Number of worker threads in the pool.");

    pyThreadPool.def("execute_simulate", [](murosim::threading::ThreadPool &thread_pool,
                                             const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) {
        pybind11::gil_scoped_release gil;
        std::list<std::function<void()>> jobs;
        for (size_t i = 0; i < simulations.size(); i++) {
            jobs.emplace_back(std::bind(&murosim::simulation::Simulation::simulate, simulations[i]));
        }
        thread_pool.execute(std::move(jobs));
    }, "Execute simulation step on all simulations in parallel.\n\n"
       "Args:\n"
       "    simulations: List of simulations to step",
       pybind11::arg("simulations"));

    pyThreadPool.def("execute_reset", [](murosim::threading::ThreadPool &thread_pool,
                                          const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) {
        pybind11::gil_scoped_release gil;
        std::list<std::function<void()>> jobs;
        for (size_t i = 0; i < simulations.size(); i++) {
            jobs.emplace_back(std::bind(&murosim::simulation::Simulation::reset, simulations[i]));
        }
        thread_pool.execute(std::move(jobs));
    }, "Reset all simulations in parallel.\n\n"
       "Args:\n"
       "    simulations: List of simulations to reset",
       pybind11::arg("simulations"));

    pyThreadPool.def("execute_planning", [](murosim::threading::ThreadPool &thread_pool,
                                             const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations) {
        pybind11::gil_scoped_release gil;
        std::list<std::function<void()>> jobs;
        for (const std::shared_ptr<murosim::simulation::Simulation>& sim : simulations) {
            jobs.emplace_back(std::bind(&murosim::simulation::Simulation::plan, sim));
        }
        thread_pool.execute(std::move(jobs));
    }, "Execute path planning on all simulations in parallel.\n\n"
       "Args:\n"
       "    simulations: List of simulations to run planning on",
       pybind11::arg("simulations"));

    pyThreadPool.def("execute_reset_by_finished", [](murosim::threading::ThreadPool &thread_pool,
                                                      const std::vector<std::shared_ptr<murosim::simulation::Simulation>> &simulations,
                                                      const size_t steps) {
        pybind11::gil_scoped_release gil;
        std::list<std::function<void()>> jobs;
        for (size_t i = 0; i < simulations.size(); i++) {
            if (simulations[i]->isFinished() || (steps != 0 && steps <= simulations[i]->getSimulatedStepCount())) {
                jobs.emplace_back(std::bind(&murosim::simulation::Simulation::reset, simulations[i]));
            }
        }
        thread_pool.execute(std::move(jobs));
    }, "Reset simulations that are finished or have exceeded step limit.\n\n"
       "Args:\n"
       "    simulations: List of simulations to potentially reset\n"
       "    steps: Maximum step count before reset (0 = only reset finished)",
       pybind11::arg("simulations"), pybind11::arg("steps") = 0);

    // Python special methods
    pyThreadPool.def("__repr__", [](const murosim::threading::ThreadPool &pool) {
        return "<ThreadPool threads=" + std::to_string(pool.numberOfThreads()) + ">";
    }, "Return a short debug string with the number of worker threads.");
}

} // namespace murosimpy
