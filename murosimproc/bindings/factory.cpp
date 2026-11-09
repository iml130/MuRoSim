// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "factory.hpp"

#include <murosimproc/factory.hpp>

namespace murosimprocpy {

void initModuleFactory(pybind11::module &m) {
    auto pyFactory = pybind11::class_<murosimproc::Factory>(m, "Factory", R"doc(
Holds a weighted pool of Generators and produces batches of worlds (or ready-to-use
Simulations) from it in parallel across a thread pool, retrying a fresh random pick if
a chosen generator fails to meet its minimum node count.

Each call to generate_data_containers()/generate_simulations() seeds itself from a
default-constructed random generator, so results are **not reproducible** from run to
run: no seed can be passed in.
)doc");

    pyFactory.def(pybind11::init<size_t, size_t>(),
        R"doc(
Constructs a factory with no generators yet (add them via add()).

Args:
    num_threads: Number of worker threads to use for batch generation; 0 (default)
        uses the number of hardware threads (or 1 if that cannot be determined).
    max_retries: Maximum number of generator picks to try per requested world before
        giving up; must be > 0.

Raises:
    ValueError: If max_retries == 0.
)doc",
        pybind11::arg("num_threads") = 0,
        pybind11::arg("max_retries") = 100);
    pyFactory.def("add", pybind11::overload_cast<std::shared_ptr<murosimproc::generators::Generator>, double>(&murosimproc::Factory::addGenerator),
        "Adds a generator to the pool with a relative selection weight.\n\n"
        "Args:\n"
        "    generator: The generator to add.\n"
        "    weight: Relative selection weight; must be > 0. Defaults to 1.0.\n\n"
        "Raises:\n"
        "    ValueError: If generator is None or weight <= 0.",
        pybind11::arg("generator"), pybind11::arg("weight") = 1.0);
    pyFactory.def("generate_data_containers", &murosimproc::Factory::generateDataContainers,
        "Generates count worlds in parallel (using num_threads threads), each from an\n"
        "independently, non-reproducibly seeded weighted-random generator pick.\n\n"
        "Args:\n"
        "    count: Number of worlds to generate.\n\n"
        "Returns:\n"
        "    A list of DataContainer objects.\n\n"
        "Raises:\n"
        "    RuntimeError: If no generator has been added, or generation fails for any\n"
        "        of the requested worlds after max_retries attempts.",
        pybind11::arg("count"), pybind11::call_guard<pybind11::gil_scoped_release>());
    pyFactory.def("generate_simulations", &murosimproc::Factory::generateSimulations,
        "Same as generate_data_containers(), additionally wrapping each result in a\n"
        "manually-initialized Simulation.\n\n"
        "Args:\n"
        "    count: Number of simulations to generate.\n"
        "    grid_scale: Metres per occupancy-grid cell, used to initialize each\n"
        "        Simulation.\n"
        "    buffer_scale: Extra clearance, in metres, added around obstacles in the\n"
        "        grid, used to initialize each Simulation.\n\n"
        "Returns:\n"
        "    A list of Simulation objects.\n\n"
        "Raises:\n"
        "    RuntimeError: If no generator has been added, or generation fails for any\n"
        "        of the requested worlds after max_retries attempts.",
        pybind11::arg("count"), pybind11::arg("grid_scale") = 0.1f, pybind11::arg("buffer_scale") = 0.2f, pybind11::call_guard<pybind11::gil_scoped_release>());
    pyFactory.def_property_readonly("num_threads", &murosimproc::Factory::getNumThreads,
        "The configured worker-thread count, or the resolved hardware-concurrency value if 0 was passed to the constructor.");
    pyFactory.def_property_readonly("max_retries", &murosimproc::Factory::getMaxRetries,
        "The configured maximum number of retries per requested world.");
}

} // namespace murosimprocpy
