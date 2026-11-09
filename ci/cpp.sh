#!/usr/bin/env bash
# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT
#
# C++ check: Release + -Werror build, full test suite via ctest. The C++
# examples are built too (they open a window, so they are never run here).
#
#   ci/cpp.sh OFF   libraries + tests, then `cmake --install` and build/run
#                   examples/cmake_consumer against the installed tree
#   ci/cpp.sh ON    the same build with the Python bindings enabled
#
# Environment: BUILD_DIR (default build-ci-<off|on>), CMAKE_BUILD_PARALLEL_LEVEL.
set -euo pipefail
cd "$(dirname "$0")/.."

bindings="${1:?usage: ci/cpp.sh ON|OFF}"
mode="$(echo "$bindings" | tr '[:upper:]' '[:lower:]')"
# Release by default; BUILD_TYPE=Debug keeps the asserts on (a throw that is
# preceded by an assert on the same condition only shows up there).
# The compiler comes from the usual CC/CXX environment (CMake honours them):
# the Clang job runs this script with CC=clang CXX=clang++.
build_type="${BUILD_TYPE:-Release}"
build_dir="${BUILD_DIR:-build-ci-${mode}-$(echo "$build_type" | tr '[:upper:]' '[:lower:]')}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}"

echo "== configure (bindings ${bindings}, ${build_type})"
cmake -S . -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE="${build_type}" \
    -DCMAKE_CXX_FLAGS=-Werror \
    -DBUILD_TESTING=ON \
    -DMUROSIM_BUILD_EXAMPLES=ON \
    -DMUROSIM_BUILD_PYTHON_BINDINGS="${bindings}"

echo "== build"
cmake --build "${build_dir}"

echo "== test"
# --no-tests=error: a bare ctest exits 0 when it finds nothing.
ctest --test-dir "${build_dir}" -j "$(nproc)" --no-tests=error --output-on-failure

if [ "${bindings}" = "OFF" ] && [ "${build_type}" = "Release" ]; then
    prefix="${build_dir}/prefix"
    echo "== install to ${prefix}"
    cmake --install "${build_dir}" --prefix "${prefix}"

    echo "== find_package consumer example"
    cmake -S examples/cmake_consumer -B "${build_dir}/consumer" \
        -DCMAKE_PREFIX_PATH="$(realpath "${prefix}")"
    cmake --build "${build_dir}/consumer"
    "${build_dir}/consumer/consumer"
fi
