<!--
Copyright Fraunhofer IML

Licensed under the MIT License.
For details on the licensing terms, see the LICENSE file.
SPDX-License-Identifier: MIT
-->

# Examples

| Example | Modules | Needs a display |
|---------|---------|-----------------|
| [`python/simulation.py`](python/simulation.py) | `murosimpy` | no |
| [`python/mir100.py`](python/mir100.py) | `murosimpy` | no |
| [`python/world_file.py`](python/world_file.py) | `murosimpy` | no |
| [`python/save_world.py`](python/save_world.py) | `murosimpy` | no |
| [`python/rl_loop.py`](python/rl_loop.py) | all three | no |
| [`python/batch.py`](python/batch.py) | all three | no |
| [`python/render.py`](python/render.py) | `murosimpy`, `murosimrlpy` | yes |
| [`visualization.cpp`](visualization.cpp) | `murosim` | yes |
| [`generators.cpp`](generators.cpp) | `murosimproc` | yes |
| [`cmake_consumer/`](cmake_consumer) | all three | no |

## Python

Install the package, then run them directly:

```bash
pip install .
python examples/python/simulation.py
python examples/python/rl_loop.py
```

`simulation.py` is the smallest complete program: build a world, add robots,
step, read the sensors. `mir100.py` drives one of the robot presets, a
differential-drive MiR100 with acceleration limits and two lidars.
`world_file.py` loads the hand-written [`python/warehouse.world`](python/warehouse.world)
(the file documents the format) and pairs starts and goals by node group;
`save_world.py` builds a world and its nodes in code, saves it and loads it
back.

`rl_loop.py` runs reinforcement-learning episodes across all three modules —
`murosimprocpy` generates the world, `murosimpy` simulates it, `murosimrlpy`
turns it into observations, actions, rewards and done flags — behind a small
`Environment` class with the `reset()`/`step()` shape a training framework
wraps; its hand-written policy slides along what the lidar sees. `batch.py`
imports world, robots, observations, rewards and policy from it and runs the
same loop over 64 simulations stepped in parallel from one thread
pool, with finished episodes reset in place, and reports the throughput.

`render.py` is the one Python example that opens a window: it replays the
`rl_loop.py` episodes through the on-screen renderer, one frame per step.
Any key ends the episode, closing the window ends the program. `render()`
also returns each frame as an RGB numpy array, top row first; MuRoSim ships
no image writer, so saving frames is a matter of handing that array to the image
library of your choice. `ci/python.sh` only compiles this example.

## C++

Off by default, since both open a window:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DMUROSIM_BUILD_EXAMPLES=ON
cmake --build build
./build/examples/murosim_example_visualization
./build/examples/murosim_example_generators [seed]
```

`visualization.cpp` walks through every `Renderer::draw()` overload, each
scene playing for a few seconds; `generators.cpp` shows a 3x3 grid of worlds
per procedural generator and waits on each. In both, a key press in the
window moves on and closing it quits. The generators draw from
`std::random_device` unless a seed is given, which makes a run repeatable;
the seed used is printed either way.

## Consuming the installed package

`cmake_consumer/` is a standalone project rather than part of this build: it
finds MuRoSim the way an external project does, which is also how `ci/cpp.sh`
checks that the install rules and exported targets are complete.

```bash
sudo cmake --install <build dir> --prefix /opt/murosim
cmake -S examples/cmake_consumer -B build-consumer -DCMAKE_PREFIX_PATH=/opt/murosim
cmake --build build-consumer && ./build-consumer/consumer
```
