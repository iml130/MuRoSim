# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.24 FATAL_ERROR)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
include(MuRoSimHelpers)

project(murosimproc
    VERSION ${MUROSIM_VERSION}
    DESCRIPTION "Multi-robot simulator procedural map generation library"
    LANGUAGES CXX
)

# CONFIG: use upstream BoostConfig.cmake (Boost >= 1.70) rather than CMake's
# FindBoost module, which CMake 3.30+ removed under policy CMP0167.
find_package(Boost REQUIRED CONFIG COMPONENTS headers)

# Boost.Geometry 1.90 is REQUIRED, not merely preferred: its rewritten
# traversal engine fixes a bg::union_ bug in system Boost <= 1.88 that splits
# a single polygon into fragments. Reproducer (graph generator, grid n=8,
# step 5.0, spacing 2.72, 8 buckets, seed 12345): 1.90 yields one polygon of
# area 1626.7; 1.83 yields two (889.7 + 172.6) and loses ~35% of the area.
# Boost.Core 1.90 is what Geometry 1.90 needs (invoke_swap.hpp).
include(FetchContent)
FetchContent_Declare(
    boost_geometry
    GIT_REPOSITORY https://github.com/boostorg/geometry.git
    GIT_TAG        boost-1.90.0
    GIT_SHALLOW    TRUE
)
FetchContent_Declare(
    boost_core
    GIT_REPOSITORY https://github.com/boostorg/core.git
    GIT_TAG        boost-1.90.0
    GIT_SHALLOW    TRUE
)
# Shadow BUILD_TESTING while adding the fetched Boost subprojects: they gate
# their own (unbuildable standalone) test suites on it.
set(_build_testing_saved ${BUILD_TESTING})
set(BUILD_TESTING OFF)
FetchContent_MakeAvailable(boost_geometry boost_core)
set(BUILD_TESTING ${_build_testing_saved})

add_library(murosimproc STATIC
    src/generators/generator.cpp
    src/factory.cpp
    # Generators
    src/generators/wfc_generator.cpp
    src/generators/maze_generator.cpp
    src/generators/graph_generator.cpp
    src/generators/line_generator.cpp
    src/generators/random_sparse_generator.cpp
    src/generators/intersection_generator.cpp
    # WFC algorithm
    src/wfc/post_processor.cpp
    src/wfc/preset.cpp
    src/wfc/tiles.cpp
    src/wfc/wfc.cpp
    src/wfc/propagator.cpp
    src/wfc/wave.cpp
    # Graph algorithms
    src/graph/algorithm.cpp
    src/graph/converter.cpp
    src/graph/filter.cpp
    # Maze algorithms
    src/maze/algorithm.cpp
    src/maze/converter.cpp
    # Utilities
    src/util/image_contour.cpp
    src/util/geometry.cpp
    src/util/image.cpp
    src/util/points.cpp
    src/util/polygon.cpp
    src/util/spatial_grid.cpp
    # Obstacle placement
    src/obstacle/convert_shapes.cpp
    src/obstacle/place_obstacles.cpp
)
murosim_configure_library(murosimproc)

# The fetched Boost::geometry target cannot be linked: it depends on
# Boost::algorithm, Boost::any, ... which the system BoostConfig does not
# provide for header-only libraries. Put the 1.90 headers on the include path
# BEFORE the system ones instead. SYSTEM: third-party headers, keep their
# warnings out of our -Wall output (-isystem still precedes /usr/include).
# PRIVATE: Boost is only used in src/, no public header includes it.
target_include_directories(murosimproc SYSTEM BEFORE PRIVATE
    ${boost_geometry_SOURCE_DIR}/include
    ${boost_core_SOURCE_DIR}/include
)
# Boost.Core 1.90 swap.hpp vs. system Boost headers
target_compile_definitions(murosimproc PRIVATE BOOST_ALLOW_DEPRECATED_HEADERS)
# GCC reports -Wmaybe-uninitialized and -Wnull-dereference inside
# Boost.Geometry's inlined templates at -O2/-O3 even though the headers are
# SYSTEM (false positives, reported at our instantiation site). Silence both
# for the sources that use Boost.Geometry only, so -Werror stays usable.
set_source_files_properties(
    src/graph/algorithm.cpp
    src/graph/converter.cpp
    src/util/polygon.cpp
    PROPERTIES COMPILE_OPTIONS
        "$<$<CXX_COMPILER_ID:GNU>:-Wno-maybe-uninitialized;-Wno-null-dereference>"
)

target_link_libraries(murosimproc
    PUBLIC MuRoSim::murosim
    # Header-only and used in src/ only: nothing for consumers of the
    # installed static library to find or link.
    PRIVATE $<BUILD_INTERFACE:Boost::headers>
)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

if(MUROSIM_BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings)
endif()
