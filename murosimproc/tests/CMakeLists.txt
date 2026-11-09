# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

find_package(GTest REQUIRED)

add_executable(murosimproctest
    main.cpp
    factory_test.cpp
    generators/generator_test.cpp
    generators/graph_generator_test.cpp
    generators/intersection_generator_test.cpp
    generators/line_generator_test.cpp
    generators/maze_generator_test.cpp
    generators/random_range_test.cpp
    generators/random_sparse_generator_test.cpp
    generators/wfc_generator_test.cpp
    graph/algorithm_test.cpp
    graph/converter_test.cpp
    graph/filter_test.cpp
    graph/graph_test.cpp
    maze/algorithm_test.cpp
    maze/converter_test.cpp
    obstacle/convert_shapes_test.cpp
    obstacle/place_obstacles_test.cpp
    obstacle/shape_distribution_test.cpp
    util/array2d_test.cpp
    util/array3d_test.cpp
    util/geometry_test.cpp
    util/image_test.cpp
    util/points_test.cpp
    util/polygon_test.cpp
    util/spatial_grid_test.cpp
    wfc/direction_test.cpp
    wfc/post_processor_test.cpp
    wfc/preset_test.cpp
    wfc/propagator_test.cpp
    wfc/tiles_test.cpp
    wfc/wave_test.cpp
    wfc/wfc_test.cpp
)
target_link_libraries(murosimproctest PRIVATE MuRoSim::murosimproc GTest::gtest)
murosim_configure_warnings(murosimproctest)
target_include_directories(murosimproctest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

include(GoogleTest)
gtest_discover_tests(murosimproctest)
