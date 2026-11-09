# MuRoSim

MuRoSim is a multi-robot simulator written in C++20 with Python bindings,
developed at [Fraunhofer IML](https://www.iml.fraunhofer.de/). It is built for
large-scale reinforcement learning of mobile robot navigation.

Features:

- Many simulations with many mobile robots each, stepped in parallel on a
  thread pool
- Fast lidar and collision computation with SIMD instructions (AVX-512, AVX,
  SSE or NEON, chosen at build time)
- Python bindings with numpy arrays, ready for deep-learning frameworks such
  as PyTorch
- RL modules that compute observations, rewards and actions for whole batches
  of robots
- Procedural world generation
- Path planning
- On-screen and headless rendering (GLFW, EGL)

This repository builds three C++ libraries and their pybind11 wrappers in one
step, producing three Python extension modules.

## Architecture

The packages are organized in two layers:

| Layer | Packages | Description |
|-------|----------|-------------|
| Core C++ | `murosim`, `murosimrl`, `murosimproc` | Simulator core, RL interface, procedural map generation |
| Python bindings | `murosimpy`, `murosimrlpy`, `murosimprocpy` | Pybind11 wrappers exposing C++ to Python |

Each Python binding module lives inside its C++ package under `<package>/bindings/`
(e.g. `murosim/bindings/` builds the `murosimpy` module). Bindings are built when
`MUROSIM_BUILD_PYTHON_BINDINGS=ON`, which is the default for the root build; pass
`-DMUROSIM_BUILD_PYTHON_BINDINGS=OFF` for a C++-only build without Python/pybind11.

## Dependencies

```bash
sudo apt install build-essential cmake pkg-config python3-dev python3-venv \
    libeigen3-dev libglfw3-dev libegl-dev libgles-dev libboost-dev
```

For running the C++ tests, also install:

```bash
sudo apt install libgtest-dev
```

### Version requirements

| Requirement | Reason |
|-------------|--------|
| **CMake >= 3.24** | The root build pins pybind11 through `FetchContent`'s `OVERRIDE_FIND_PACKAGE` |
| **Boost >= 1.70** | `find_package(Boost CONFIG)` resolves upstream's `BoostConfig.cmake` |
| C++20 compiler | `CMAKE_CXX_STANDARD 20`, no compiler extensions; CI tests GCC 13 and Clang 18 |
| Eigen 3.4 or 5 | `find_package(Eigen3 CONFIG)`; CI tests 3.4, Eigen 5 is supported but not covered by CI |
| Python >= 3.12 | `requires-python` in `pyproject.toml`; the bindings need numpy at array calls |
| Linux | the renderer uses EGL/GLES and GLFW; no Windows or macOS build is provided |

Ubuntu 24.04 is the supported platform and the one CI runs on; its packages
satisfy all of these (GCC 13, CMake 3.28, Boost 1.83, Eigen 3.4, Python 3.12).
Other distributions may work but are not tested. Older ones may fall short —
Ubuntu 22.04 ships CMake 3.22 and Python 3.10 — in which case install Python 3.12
(e.g. from the deadsnakes PPA, or use a conda environment) and a newer CMake
before building:

```bash
pip install cmake
```

All packages declare the same CMake 3.24 minimum, including a standalone
configure of `murosim` (`cmake -S murosim -B build`).

## Install

```bash
git clone https://github.com/iml130/MuRoSim.git
cd MuRoSim
python3 -m venv .venv
. .venv/bin/activate
MUROSIM_NATIVE_ARCH=1 pip install .
```

Ubuntu 24.04 refuses `pip install` into the system Python (PEP 668,
"externally-managed-environment"), hence the virtual environment; activate it
with `. .venv/bin/activate` in every new shell before using MuRoSim.

Configuring the build, with pip as here or with CMake below, downloads
Boost.Geometry and Boost.Core (and, for the Python bindings, pybind11) from
GitHub through CMake's `FetchContent`, so the first build needs internet access.

MuRoSim is distributed through GitHub, not PyPI. To install it without cloning,
pip can build it straight from the repository (same dependencies, inside a
virtual environment):

```bash
pip install "git+https://github.com/iml130/MuRoSim.git"
```

The root `setup.py` builds all three CMake extensions (`murosimpy`, `murosimrlpy`, `murosimprocpy`) in one step via `cmake_build.py`.

### C++ libraries

To use the C++ libraries directly, install them with CMake into their own
directory and consume the `MuRoSim` package:

```cmake
find_package(MuRoSim 1.0 CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE MuRoSim::murosim MuRoSim::murosimrl MuRoSim::murosimproc)
```

To build MuRoSim and install it into `/opt/murosim`:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DMUROSIM_BUILD_PYTHON_BINDINGS=OFF
cmake --build build --parallel "$(nproc)"
sudo cmake --install build --prefix /opt/murosim
```

`/opt/murosim` is not on CMake's default search path, so `find_package` needs
it, either per project with `-DCMAKE_PREFIX_PATH=/opt/murosim` or once for all
projects in your `~/.bashrc`:

```bash
echo 'export CMAKE_PREFIX_PATH="/opt/murosim${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"' >> ~/.bashrc
```

CMake has no uninstall command; as everything lives in one directory,
`sudo rm -rf /opt/murosim` removes the installation.

A complete minimal consumer project is in [`examples/cmake_consumer`](examples/cmake_consumer).
Tests are built with `-DBUILD_TESTING=ON` and run with `ctest --test-dir build`.
The Python smoke tests in `tests/python/` run against the installed modules with
`pytest` (`pip install ".[dev]"` first); they check the binding layer, not the
simulator logic, which the C++ suites cover.

### Type stubs

The wheel ships PEP 561 stub packages (`murosimpy-stubs`, `murosimrlpy-stubs`,
`murosimprocpy-stubs`) so IDEs and type checkers see signatures and docstrings
for the extension modules. They are generated from the built modules and
committed under `stubs/`; after changing the bindings, regenerate them with

```bash
pip install "pybind11-stubgen>=3"
python tools/generate_stubs.py        # --check verifies they are current
```

### Environment variables

| Variable | Description |
|----------|-------------|
| `MUROSIM_NATIVE_ARCH=1` | Enable `-march=native` optimization |
| `CMAKE_BUILD_PARALLEL_LEVEL` | Number of parallel build jobs |

## Usage

```python
import math
import murosimpy

simulation = murosimpy.core.Simulation(murosimpy.models.world.empty(10.0, 1.0))

builder = murosimpy.core.RobotBuilder()
builder.set_name("robot_0")
builder.set_actuator_holonomic_simple(-1.0, 1.0, 1.0, 1.0)
builder.set_shape_robot_rectangle(0.3, 0.3)
builder.add_lidar(murosimpy.geometry.Transform(), 2.0 * math.pi, 32, 0.05, 10.0, 10.0)

robot = builder.build()
robot.pose = (2.0, 2.0, 0.0)   # x, y, rotation
robot.goal = (8.0, 2.0, 0.3)   # x, y, goal radius
simulation.add_robot(robot)
simulation.reset()

while not simulation.finished:
    robot.command_velocity = [1.0, 0.0, 0.0]   # in the robot's own frame
    simulation.step()

print(robot.pose, robot.lidar_distance().min())
```

Lidar readings follow ROS [REP 117](https://www.ros.org/reps/rep-0117.html):
no hit within `range_max` reads `inf`, a hit closer than `range_min` reads
`-inf`. `lidar_distance(0, True)` and the lidar observations in `murosimrlpy`
clamp them onto the range, so their values are always finite.

### Reproducibility

A simulation is deterministic for a given `SimulationConfig.seed`, and each
`murosimproc` generator returns the same world for the same seed. The `Factory`
is not reproducible: it seeds every world randomly. For reproducible training
sets, call the generators with your own seeds.

### Runnable examples

Runnable programs live in [`examples/`](examples): headless Python
simulations (a minimal one, a robot preset, a world file loaded and saved),
reinforcement-learning episodes across all three modules, the same loop over
a batch of simulations stepped in parallel, the same episodes in an on-screen
window, two interactive C++ visualizations and a `find_package(MuRoSim)`
consumer project. See [`examples/README.md`](examples/README.md).

## Feedback

Bug reports, reproducers and feature requests are welcome as
[GitHub issues](https://github.com/iml130/MuRoSim/issues). Pull requests are
currently not accepted, as there is no process yet for taking in code from
outside Fraunhofer IML. Forks are welcome under the MIT license.

Please report security problems privately to
<christian.jestel@iml.fraunhofer.de> rather than in a public issue.

## Citation

If you use MuRoSim in your research, please cite the ICRA 2024 paper:

```bibtex
@inproceedings{jestel2024murosim,
  author    = {Jestel, Christian and R{\"o}sner, Karol and Dietz, Niklas and Bach, Nicolas and E{\ss}er, Julian and Finke, Jan and Urbann, Oliver},
  title     = {{MuRoSim} -- A Fast and Efficient Multi-Robot Simulation for Learning-based Navigation},
  booktitle = {2024 IEEE International Conference on Robotics and Automation (ICRA)},
  address   = {Yokohama, Japan},
  pages     = {16881--16887},
  year      = {2024},
  doi       = {10.1109/ICRA57147.2024.10610375}
}
```

If you use the procedural map generators of `murosimproc`, please also cite:

```bibtex
@inproceedings{jestel2026procedural,
  author    = {Jestel, Christian and Bach, Nicolas and Wiedemann, Marvin and Finke, Jan and Detzner, Peter},
  title     = {Beyond Specialization: Robust Reinforcement Learning Navigation Via Procedural Map Generators},
  booktitle = {2026 IEEE 31st International Conference on Emerging Technologies and Factory Automation (ETFA)},
  address   = {V{\"a}ster{\aa}s, Sweden},
  pages     = {1--8},
  year      = {2026},
  doi       = {10.1109/ETFA68377.2026.11725096}
}
```

## License

MIT — see [LICENSE](LICENSE) for details.

MuRoSim builds against third-party components that carry their own licenses,
and part of `murosimproc` is adapted from another MIT-licensed project; both
are documented in [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).

The robot presets are named after real products (MiR100, RoboMaster, Kobuki).
Those names are trademarks of their respective owners. The presets approximate
published dimensions and limits; they are neither affiliated with nor endorsed
by those companies, and contain no vendor-supplied data.
