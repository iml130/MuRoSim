# Third-Party Licenses

MuRoSim is released under the MIT License (see [LICENSE](LICENSE)). It uses the
third-party components listed below, and parts of its own source are adapted from
or based on a third-party project (see [Adapted source](#adapted-source)). The components in
the table are not vendored into this repository: they are fetched at CMake
configure time or resolved from the system.

| Component | Version | License | How obtained | In binaries |
|---|---|---|---|---|
| [pybind11](https://github.com/pybind/pybind11) | v3.1.0 | [BSD-3-Clause](https://github.com/pybind/pybind11/blob/v3.1.0/LICENSE) | FetchContent | yes |
| [Boost.Geometry](https://github.com/boostorg/geometry) | 1.90.0 | [BSL-1.0](https://www.boost.org/LICENSE_1_0.txt) | FetchContent | yes (header-only) |
| [Boost.Core](https://github.com/boostorg/core) | 1.90.0 | [BSL-1.0](https://www.boost.org/LICENSE_1_0.txt) | FetchContent | yes (header-only) |
| Boost headers | system | BSL-1.0 | find_package | yes (header-only) |
| [Eigen](https://eigen.tuxfamily.org) | system (3.4 or 5) | MPL-2.0 | find_package | yes (header-only) |
| [GLFW 3](https://www.glfw.org) | system | Zlib | pkg-config | linked |
| EGL / OpenGL ES 2 (Mesa, Khronos headers) | system | MIT | pkg-config | linked |
| [GoogleTest](https://github.com/google/googletest) | system | BSD-3-Clause | tests only | no |

Notices:

- pybind11: binary wheels of `murosimpy`, `murosimrlpy` and `murosimprocpy`
  contain pybind11-derived code. Its license, which binary redistributions must
  reproduce, follows in full under [pybind11 license](#pybind11-license); this
  file ships in every wheel and source distribution.
- Boost Software License: attribution is required for source distributions only.
- Eigen (MPL-2.0) is used unmodified; its file-level copyleft does not extend to
  MuRoSim's own sources. The binaries contain code compiled from Eigen's
  headers, whose source code is available at
  <https://gitlab.com/libeigen/eigen>; they are built against the system's
  release (3.4.0 on Ubuntu 24.04, the CI platform). The license text is at
  <https://www.mozilla.org/MPL/2.0/>.

All of the above are compatible with distribution under the MIT License.

## pybind11 license

pybind11 v3.1.0, from its `LICENSE` file:

```
Copyright (c) 2016 Wenzel Jakob <wenzel.jakob@epfl.ch>, All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Adapted source

The Wave Function Collapse implementation in `murosimproc/src/wfc/` and
`murosimproc/include/murosimproc/wfc/`, together with the `Array2d` and `Array3d`
containers in `murosimproc/include/murosimproc/util/`, is adapted from or based
on [fast-wfc](https://github.com/math-fehr/fast-wfc). Every file that contains
adapted code says so in its header and points to this section; the MuRoSim-specific
parts (the presets and the post-processor) are original. They are distributed
under the MIT License, like the rest of MuRoSim; the upstream notice is
reproduced here as that licence requires.

```
MIT License

Copyright (c) 2018-2019 Mathieu Fehr and Nathanaël Courant

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

fast-wfc is itself an implementation of the Wave Function Collapse algorithm
devised by Maxim Gumin. His original implementation,
[mxgmn/WaveFunctionCollapse](https://github.com/mxgmn/WaveFunctionCollapse)
(Copyright (c) 2016 Maxim Gumin), is also released under the MIT License. No
code from it is used here; the credit is given because the algorithm is his.
