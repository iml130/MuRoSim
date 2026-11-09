# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

# Single source of truth for the MuRoSim version.
#
# The version lives in the VERSION file at the repository root and nowhere else.
# Every package reads it through this module, so a standalone configure of any
# one package reports the same number as the full build, and a release is a
# one-line edit to VERSION.

if(NOT DEFINED MUROSIM_VERSION)
    set(_murosim_version_file "${CMAKE_CURRENT_LIST_DIR}/../VERSION")
    if(NOT EXISTS "${_murosim_version_file}")
        message(FATAL_ERROR
            "MuRoSim version file not found: ${_murosim_version_file}")
    endif()

    file(READ "${_murosim_version_file}" _murosim_version)
    string(STRIP "${_murosim_version}" _murosim_version)

    if(NOT _murosim_version MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
        message(FATAL_ERROR
            "Malformed version '${_murosim_version}' in ${_murosim_version_file}; "
            "expected MAJOR.MINOR.PATCH")
    endif()

    set(MUROSIM_VERSION "${_murosim_version}"
        CACHE INTERNAL "MuRoSim version, read from the VERSION file")

    unset(_murosim_version_file)
    unset(_murosim_version)
endif()

# murosim_add_version_header(<target> <macro_prefix>)
#
# Generates <binary dir>/generated/version.hpp for <target> from
# cmake/version.hpp.in, defining <macro_prefix>_VERSION along with the
# _MAJOR/_MINOR/_PATCH components, and puts the generated directory on the
# target's PRIVATE include path so it can `#include "version.hpp"`.
function(murosim_add_version_header target macro_prefix)
    if(NOT TARGET ${target})
        message(FATAL_ERROR
            "murosim_add_version_header: no such target '${target}'")
    endif()

    # Consumed by version.hpp.in via configure_file(@ONLY).
    set(MUROSIM_VERSION_PREFIX "${macro_prefix}")
    string(REPLACE "." ";" _parts "${MUROSIM_VERSION}")
    list(GET _parts 0 MUROSIM_VERSION_MAJOR)
    list(GET _parts 1 MUROSIM_VERSION_MINOR)
    list(GET _parts 2 MUROSIM_VERSION_PATCH)

    set(_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
    configure_file(
        "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/version.hpp.in"
        "${_generated_dir}/version.hpp"
        @ONLY
    )

    target_include_directories(${target} PRIVATE "${_generated_dir}")
endfunction()
