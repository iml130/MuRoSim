# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

"""Generate the PEP 561 type stubs (stubs/<module>-stubs/) for the three
extension modules with pybind11-stubgen, or verify that the committed stubs
are up to date.

    python tools/generate_stubs.py           # regenerate stubs/ in place
    python tools/generate_stubs.py --check   # exit 1 if stubs/ is stale (CI)

The built modules must be importable (pip install . or PYTHONPATH to a build
tree) and pybind11-stubgen installed (pip install pybind11-stubgen).
"""

import argparse
import filecmp
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

MODULES = ["murosimpy", "murosimrlpy", "murosimprocpy"]
STUBS_DIR = Path(__file__).resolve().parent.parent / "stubs"

# The license header every source file carries, prepended to each stub.
LICENSE_HEADER = (
    "# Copyright Fraunhofer IML\n"
    "#\n"
    "# Licensed under the MIT License.\n"
    "# For details on the licensing terms, see the LICENSE file.\n"
    "# SPDX-License-Identifier: MIT\n"
    "\n"
)

# Enums defined in one submodule but used as default values in others; stubgen
# needs to be told where they live to render those defaults.
ENUM_LOCATIONS = [
    "LidarNoiseType:murosimpy.core",
    "PlannerType:murosimpy.definitions",
]


def generate(out_dir: Path) -> None:
    for module in MODULES:
        cmd = [
            sys.executable, "-m", "pybind11_stubgen", module,
            "--root-suffix=-stubs",
            "--exit-code",
            "-o", str(out_dir),
        ]
        for loc in ENUM_LOCATIONS:
            cmd += ["--enum-class-locations", loc]
        # Run from a neutral directory so the source tree cannot shadow the modules.
        subprocess.run(cmd, check=True, cwd=tempfile.gettempdir())
    normalise(out_dir)


def normalise(out_dir: Path) -> None:
    """Make the generated stubs independent of the generating environment.

    Strips the trailing whitespace stubgen leaves on blank docstring lines,
    rewrites `numpy.bool` (which exists only on numpy >= 2, where it is an alias
    of `numpy.bool_`) to `numpy.bool_`, which is valid on every numpy this
    package supports, and prepends the license header. Applied to every
    generated tree, so the committed stubs and `--check` always compare equal.
    """
    for path in sorted(out_dir.rglob("*.pyi")):
        text = path.read_text(encoding="utf-8")
        out = "".join(line.rstrip() + "\n" for line in text.splitlines())
        out = out.replace("numpy.bool]", "numpy.bool_]")
        if not out.startswith(LICENSE_HEADER):
            out = LICENSE_HEADER + out
        if out != text:
            path.write_text(out, encoding="utf-8")


def differs(a: Path, b: Path) -> list[str]:
    """Return the relative paths that differ between two stub trees."""
    diff = []
    cmp = filecmp.dircmp(a, b)
    def walk(c: filecmp.dircmp, prefix: str) -> None:
        for name in c.left_only + c.right_only + c.diff_files:
            # setuptools writes its egg-info next to the stub packages on an
            # in-tree `pip install .` (package_dir points at stubs/)
            if not name.endswith(".egg-info"):
                diff.append(prefix + name)
        for name, sub in c.subdirs.items():
            walk(sub, prefix + name + "/")
    walk(cmp, "")
    return sorted(diff)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--check", action="store_true",
                        help="do not write; fail if stubs/ is not up to date")
    args = parser.parse_args()

    # Both modes generate into a temporary tree; stubs/ is replaced only once
    # every module succeeded, so a failing run (pybind11-stubgen missing, a
    # module not importable) leaves the committed stubs untouched.
    with tempfile.TemporaryDirectory() as tmp:
        try:
            generate(Path(tmp))
        except subprocess.CalledProcessError as error:
            print(f"pybind11-stubgen failed (exit code {error.returncode}); {STUBS_DIR} was left unchanged.")
            return 1

        if args.check:
            stale = differs(STUBS_DIR, Path(tmp))
            if stale:
                print("Type stubs are out of date; run: python tools/generate_stubs.py")
                for path in stale:
                    print("  ", path)
                return 1
            print("Type stubs are up to date.")
            return 0

        if STUBS_DIR.exists():
            shutil.rmtree(STUBS_DIR)
        shutil.copytree(tmp, STUBS_DIR)
    print(f"Stubs written to {STUBS_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
