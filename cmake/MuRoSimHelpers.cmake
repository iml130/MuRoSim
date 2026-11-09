# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

# Shared target setup for the three MuRoSim packages, so that murosim,
# murosimrl and murosimproc are configured identically.

include_guard(GLOBAL)
include(MuRoSimVersion)

# murosim_configure_warnings(<target>)
#
# The one warning set for every target we compile (libraries, Python modules,
# tests, demos): -Wall -Wextra -Wpedantic plus the checks that are free on this
# tree and catch real mistakes (a base without virtual destructor, shadowed
# members, float silently promoted to double, C casts, implicit narrowing
# and sign changes). CI adds -Werror.
function(murosim_configure_warnings target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic
            -Wshadow -Wnon-virtual-dtor -Woverloaded-virtual -Wold-style-cast
            -Wdouble-promotion -Wnull-dereference -Wimplicit-fallthrough
            -Wcast-align -Wextra-semi -Wconversion -Wsign-conversion
            $<$<CXX_COMPILER_ID:GNU>:-Wuseless-cast -Wduplicated-cond -Wlogical-op>
        )
    elseif(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    endif()
endfunction()

# murosim_configure_library(<target>)
#
# Applies the common settings to a library created with add_library():
#   - MuRoSim::<target> alias, the same name consumers get from find_package
#   - installs the library and its include/ tree into the MuRoSimTargets export
#   - C++20 as a PUBLIC compile feature (propagates to consumers)
#   - include/ as the public include directory
#   - PIC and hidden visibility (the libraries end up inside Python modules)
#   - warnings
function(murosim_configure_library target)
    if(NOT TARGET ${target})
        message(FATAL_ERROR "murosim_configure_library: no such target '${target}'")
    endif()

    add_library(MuRoSim::${target} ALIAS ${target})

    include(GNUInstallDirs)
    target_compile_features(${target} PUBLIC cxx_std_20)

    target_include_directories(${target}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

    set_target_properties(${target} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
    )

    murosim_configure_warnings(${target})

    install(TARGETS ${target} EXPORT MuRoSimTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endfunction()

# murosim_add_python_module(<name>
#     MACRO_PREFIX <PREFIX>
#     SOURCES <src>...
#     LINK <target>...)
#
# Creates the pybind11 extension module <name>, generates its version.hpp
# (exposing <PREFIX>_VERSION for m.attr("__version__")), links it PRIVATEly
# against LINK with the shared warning set, and adds a <name>_stubs target
# that runs pybind11-stubgen.
#
# The module is a top-level extension (import <name>), not a package: setup.py
# collects the built .so directly, so no __init__.py is needed.
function(murosim_add_python_module name)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "" "MACRO_PREFIX" "SOURCES;LINK")
    if(NOT ARG_MACRO_PREFIX OR NOT ARG_SOURCES)
        message(FATAL_ERROR
            "murosim_add_python_module(${name}): MACRO_PREFIX and SOURCES are required")
    endif()

    # pybind11 locates Python itself (Development.Module, which also works on
    # manylinux where no libpython is present). The root build provides
    # pybind11 through FetchContent; standalone builds need it installed.
    find_package(pybind11 CONFIG REQUIRED)

    pybind11_add_module(${name} ${ARG_SOURCES})
    murosim_add_version_header(${name} ${ARG_MACRO_PREFIX})
    target_link_libraries(${name} PRIVATE ${ARG_LINK})
    murosim_configure_warnings(${name})

    add_custom_target(${name}_stubs
        COMMAND ${Python_EXECUTABLE} -m pybind11_stubgen ${name}
                -o ${CMAKE_CURRENT_SOURCE_DIR}/stubs
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${name}
        COMMENT "Generating Python type stubs for ${name}"
    )

    message(STATUS "${name}: Python module ${MUROSIM_VERSION} (${Python_EXECUTABLE})")
endfunction()
