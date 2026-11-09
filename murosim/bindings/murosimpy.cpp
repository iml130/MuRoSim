// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "common.hpp"
#include "version.hpp"

namespace murosimpy {

// Forward declarations of all init functions
// Core module
void initCoreRobot(pybind11::module &m);
void initCoreRobotBuilder(pybind11::module &m);
void initCoreSimulation(pybind11::module &m);
void initCoreThreadPool(pybind11::module &m);

// Geometry module
void initGeometryTransform(pybind11::module &m);
void initGeometryLine(pybind11::module &m);
void initGeometryCircle(pybind11::module &m);
void initGeometryShape(pybind11::module &m);
void initGeometryWorld(pybind11::module &m);
void initGeometryPrimitives(pybind11::module &m);

// Navigation module
void initNavigationNode(pybind11::module &m);
void initNavigationNodeContainer(pybind11::module &m);

// Models module
void initModelsRobotFactory(pybind11::module &m);
void initModelsWorldFactory(pybind11::module &m);

// Containers module
void initContainersSimulationContainer(pybind11::module &m);
void initContainersRobotContainer(pybind11::module &m);

// Rendering module
void initRenderingOfflineRenderer(pybind11::module &m);
void initRenderingOnlineRenderer(pybind11::module &m);

// IO module
void initIODataContainer(pybind11::module &m);
void initIOFunctions(pybind11::module &m);

// Definitions module
void initModuleDefinitions(pybind11::module &m);

PYBIND11_MODULE(murosimpy, m) {
    m.doc() = "Python bindings for MuRoSim - Multi-Robot Simulation.\n\n"
              "MuRoSim provides efficient simulation of multiple robots with\n"
              "lidar sensors, collision detection, and path planning capabilities.";

    m.attr("__version__") = MUROSIMPY_VERSION;

    // Create submodules
    auto pyModuleDefinitions = m.def_submodule("definitions",
        "Enum definitions for robot states, collision status, actuator types, etc.");
    auto pyModuleCore = m.def_submodule("core",
        "Core simulation components: Robot, Simulation, and ThreadPool.");
    auto pyModuleGeometry = m.def_submodule("geometry",
        "Geometric primitives, transformations, and world representation.");
    auto pyModuleGeometryPrimitives = pyModuleGeometry.def_submodule("primitives",
        "Primitive geometric shapes: triangle, rectangle, square.");
    auto pyModuleNavigation = m.def_submodule("navigation",
        "Navigation nodes for goal specification and path planning.");
    auto pyModuleModels = m.def_submodule("models",
        "Predefined robot and world models.");
    auto pyModuleModelsRobot = pyModuleModels.def_submodule("robot",
        "Factory functions for creating predefined robot models.");
    auto pyModuleModelsWorld = pyModuleModels.def_submodule("world",
        "Factory functions for creating predefined world geometries.");
    auto pyModuleContainers = m.def_submodule("containers",
        "Container types for managing multiple simulations or robots.");
    auto pyModuleRendering = m.def_submodule("rendering",
        "Rendering and visualization tools for offline and online display.");
    auto pyModuleIO = m.def_submodule("io",
        "File I/O for world data and data containers.");

    // Initialize modules in dependency order
    // 1. Definitions first (no dependencies)
    initModuleDefinitions(pyModuleDefinitions);

    // 2. Geometry module (depends on definitions for enums in transforms)
    initGeometryTransform(pyModuleGeometry);
    initGeometryLine(pyModuleGeometry);
    initGeometryCircle(pyModuleGeometry);
    initGeometryShape(pyModuleGeometry);
    initGeometryWorld(pyModuleGeometry);
    initGeometryPrimitives(pyModuleGeometryPrimitives);

    // 3. Navigation module (depends on geometry)
    initNavigationNode(pyModuleNavigation);
    initNavigationNodeContainer(pyModuleNavigation);

    // 4. IO module (depends on geometry and navigation)
    initIODataContainer(pyModuleIO);
    initIOFunctions(pyModuleIO);

    // 5. Core module (depends on geometry, navigation, and io)
    // Order matters for the generated signatures: a type must be bound before
    // any method that takes or returns it. RobotContainer is used by Simulation,
    // SimulationContainer by ThreadPool.
    initCoreRobot(pyModuleCore);
    initCoreRobotBuilder(pyModuleCore);
    initContainersRobotContainer(pyModuleContainers);
    initCoreSimulation(pyModuleCore);
    initContainersSimulationContainer(pyModuleContainers);
    initCoreThreadPool(pyModuleCore);

    // 6. Models module (depends on core and io)
    initModelsRobotFactory(pyModuleModelsRobot);
    initModelsWorldFactory(pyModuleModelsWorld);

    // 8. Rendering module (depends on core, geometry, and io)
    initRenderingOfflineRenderer(pyModuleRendering);
    initRenderingOnlineRenderer(pyModuleRendering);
}

} // namespace murosimpy
