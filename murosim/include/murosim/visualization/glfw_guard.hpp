// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef MUROSIM_VISUALIZATION_GLFW_GUARD_HPP_
#define MUROSIM_VISUALIZATION_GLFW_GUARD_HPP_

#include <stdexcept>
#include <GLFW/glfw3.h>

namespace murosim::visualization {

/// Process-wide RAII guard around glfwInit()/glfwTerminate(). GLFW must be
/// initialized before any window is created and must be terminated at most
/// once; RendererGlfw calls GlfwGuard::instance() to ensure GLFW is ready
/// without each renderer owning init/terminate itself, since multiple
/// RendererGlfw instances may coexist. The singleton is constructed on first
/// use and torn down at static-destruction time (function-local static, so
/// construction is thread-safe since C++11).
class GlfwGuard {
public:
    /// Returns the process-wide guard, initializing GLFW on first call.
    /// @throws std::runtime_error if glfwInit() fails.
    static GlfwGuard& instance() {
        static GlfwGuard guard; // thread-safe since C++11
        return guard;
    }

private:
    GlfwGuard() {
        if (!glfwInit()) {
            throw std::runtime_error("[GlfwGuard]: Failed to initialize GLFW.");
        }
    }

    ~GlfwGuard() {
        glfwTerminate();
    }

    GlfwGuard(const GlfwGuard&) = delete;
    GlfwGuard& operator=(const GlfwGuard&) = delete;
};

} // namespace murosim::visualization

#endif // MUROSIM_VISUALIZATION_GLFW_GUARD_HPP_
