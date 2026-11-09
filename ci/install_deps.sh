#!/usr/bin/env bash
# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT
#
# Installs the build dependencies on Ubuntu (24.04). Works as root
# (container images) and as a sudo-capable user (dev machines).
set -euo pipefail
cd "$(dirname "$0")/.."

SUDO=""
if [ "$(id -u)" -ne 0 ]; then SUDO="sudo"; fi

export DEBIAN_FRONTEND=noninteractive
$SUDO apt-get update
# git + ca-certificates: FetchContent clones over https; python3-venv: ci/python.sh
$SUDO apt-get install -y --no-install-recommends \
    git ca-certificates python3-venv \
    $(grep -vE '^#|^$' dependencies.txt)
