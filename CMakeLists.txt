# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

# 3.24 needed for FetchContent OVERRIDE_FIND_PACKAGE
cmake_minimum_required(VERSION 3.24 FATAL_ERROR)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(MuRoSimHelpers)

project(murosimpackages VERSION ${MUROSIM_VERSION} LANGUAGES CXX)

option(MUROSIM_BUILD_PYTHON_BINDINGS "Build the pybind11 Python bindings" ON)
option(MUROSIM_BUILD_EXAMPLES "Build the C++ examples in examples/" OFF)
option(BUILD_TESTING "Build the GoogleTest suites" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# No C++20 modules: keep CMake >= 3.28 from scanning every source with
# clang-scan-deps under Clang (an extra tool that is not installed by default).
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

if(MUROSIM_BUILD_PYTHON_BINDINGS)
    # Pin pybind11 via FetchContent to avoid system package mismatches that can
    # cause random segfaults with numpy/Python version combinations.
    # Modern FindPython with Development.Module: no libpython needed, which is
    # what manylinux wheel builds require.
    set(PYBIND11_FINDPYTHON NEW)
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v3.1.0
        GIT_SHALLOW    TRUE
        OVERRIDE_FIND_PACKAGE
    )
    FetchContent_MakeAvailable(pybind11)
endif()

# Deliberately enable_testing() rather than include(CTest): the latter forces
# BUILD_TESTING ON by default and would make every `pip install .` compile the
# test suites. Without this call ctest silently finds no tests.
enable_testing()

add_subdirectory(murosim)
add_subdirectory(murosimrl)
add_subdirectory(murosimproc)

if(MUROSIM_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# `cmake --install` for C++ consumers: find_package(MuRoSim CONFIG) then link
# MuRoSim::murosim, MuRoSim::murosimrl, MuRoSim::murosimproc.
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(MUROSIM_CMAKE_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/MuRoSim)

install(EXPORT MuRoSimTargets
    NAMESPACE MuRoSim::
    DESTINATION ${MUROSIM_CMAKE_INSTALL_DIR}
)
configure_package_config_file(cmake/MuRoSimConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/MuRoSimConfig.cmake
    INSTALL_DESTINATION ${MUROSIM_CMAKE_INSTALL_DIR}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/MuRoSimConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/MuRoSimConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/MuRoSimConfigVersion.cmake
    DESTINATION ${MUROSIM_CMAKE_INSTALL_DIR}
)
