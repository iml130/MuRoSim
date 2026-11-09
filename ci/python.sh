#!/usr/bin/env bash
# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT
#
# Python check: pip install from the source tree, then build an sdist and
# install from that (proves MANIFEST.in ships everything the build needs).
# Both installs are import-checked from outside the source tree.
#
# Environment: VENV_DIR (default: fresh venv in a temp dir; an existing venv
# with pip is reused), CMAKE_BUILD_PARALLEL_LEVEL.
set -euo pipefail
cd "$(dirname "$0")/.."

venv="${VENV_DIR:-$(mktemp -d)/venv}"
if [ ! -x "${venv}/bin/python" ]; then
    echo "== create venv ${venv}"
    python3 -m venv "${venv}"
fi
python="${venv}/bin/python"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}"

check_import() {
    # From a temp dir so the source tree cannot shadow the installed modules.
    (cd "$(mktemp -d)" && "${python}" - <<'PY'
import murosimpy, murosimrlpy, murosimprocpy
for m in (murosimpy, murosimrlpy, murosimprocpy):
    print(f"  {m.__name__} {m.__version__}  ({m.__file__})")
assert murosimpy.__version__ == murosimrlpy.__version__ == murosimprocpy.__version__
PY
    )
}

echo "== pip install from the source tree"
"${python}" -m pip install --quiet .
check_import

echo "== type stubs up to date?"
"${python}" -m pip install --quiet "pybind11-stubgen>=3"
"${python}" tools/generate_stubs.py --check

echo "== build sdist"
"${python}" -m pip install --quiet build
rm -rf dist
"${python}" -m build --sdist

echo "== install from sdist"
"${python}" -m pip install --quiet --force-reinstall --no-deps dist/*.tar.gz
check_import

echo "== python smoke tests"
# Against the installed modules (tests/python has no package of its own, so
# nothing in the source tree can shadow them). The rendering tests skip
# when no OpenGL ES context is available.
"${python}" -m pip install --quiet pytest
"${python}" -m pytest tests/python -q -rs

echo "== python examples"
# Headless and quick; keeps the examples from rotting as the API changes.
# render.py opens a window, so it is only compiled here.
for example in examples/python/*.py; do
    echo "-- ${example}"
    if [ "$(basename "${example}")" = "render.py" ]; then
        "${python}" -m py_compile "${example}"
        continue
    fi
    "${python}" "${example}" > /dev/null
done
