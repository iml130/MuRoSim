# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""
Shared CMake build utilities for murosim Python bindings.

This module provides CMakeExtension and CMakeBuild classes for building
pybind11-based Python extensions using CMake.

Environment variables:
    MUROSIM_NATIVE_ARCH: Set to "1" or "ON" to enable -march=native optimization.
                         Default is OFF for portable wheels.
    CMAKE_BUILD_PARALLEL_LEVEL: Number of parallel build jobs.
    CMAKE_GENERATOR: Override the CMake generator (e.g., "Ninja").
"""

import os
import sys
import subprocess
from pathlib import Path

from setuptools import Extension
from setuptools.command.build_ext import build_ext


# Convert distutils Windows platform specifiers to CMake -A arguments
PLAT_TO_CMAKE = {
    "win32": "Win32",
    "win-amd64": "x64",
    "win-arm32": "ARM",
    "win-arm64": "ARM64",
}


class CMakeExtension(Extension):
    """A CMake-based extension module.

    Args:
        name: The extension name (e.g., "murosimpy").
        sourcedir: Path to the CMakeLists.txt directory. Defaults to current directory.
    """

    def __init__(self, name: str, sourcedir: str = "") -> None:
        super().__init__(name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    """Custom build_ext command that runs CMake to build the extension."""

    def build_extension(self, ext: CMakeExtension) -> None:
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # Required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cfg = "Debug" if self.debug else "Release"

        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        # Base CMake arguments
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPython_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
        ]

        # Handle native architecture optimization
        # Only enable with explicit environment variable for portable wheels.
        # The flags are passed on every configure, not only for native builds:
        # CMake caches CMAKE_CXX_FLAGS, so leaving them out would silently keep
        # -march=native from an earlier native build in a reused build directory
        # and produce binaries that only run on that CPU. CXXFLAGS from the
        # environment is kept, which CMake would otherwise read on the first
        # configure only. MSVC keeps its default flags (e.g. /EHsc), which an
        # explicit CMAKE_CXX_FLAGS would replace.
        native_arch = os.environ.get("MUROSIM_NATIVE_ARCH", "").upper()
        if self.compiler.compiler_type != "msvc":
            cxx_flags = os.environ.get("CXXFLAGS", "")
            if native_arch in ("1", "ON", "TRUE", "YES"):
                cxx_flags += " -march=native -mtune=native"
            cmake_args.append(f"-DCMAKE_CXX_FLAGS={cxx_flags.strip()}")

        build_args = []

        if self.compiler.compiler_type == "msvc":
            # MSVC-specific configuration
            single_config = any(x in cmake_generator for x in {"NMake", "Ninja"})
            contains_arch = any(x in cmake_generator for x in {"ARM", "Win64"})

            if not single_config and not contains_arch:
                cmake_args.append(f"-A{PLAT_TO_CMAKE[self.plat_name]}")

            if not single_config:
                cmake_args.append(
                    f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}"
                )
                build_args.append(f"--config={cfg}")

        # Set parallel build level
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            if hasattr(self, "parallel") and self.parallel:
                build_args.append(f"-j{self.parallel}")
            else:
                # Default to number of CPUs
                import multiprocessing
                build_args.append(f"-j{multiprocessing.cpu_count()}")

        build_temp = Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)

        subprocess.check_call(
            ["cmake", ext.sourcedir, *cmake_args],
            cwd=build_temp
        )
        subprocess.check_call(
            ["cmake", "--build", ".", "--target", ext.name, *build_args],
            cwd=build_temp
        )
