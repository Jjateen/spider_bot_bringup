#!/usr/bin/env bash
#
# install_open3d.sh — Install dependencies for the AHRS visualizer.
#
# Installs:
#   - Python: numpy, scipy, pyyaml
#   - Open3D: official (x86_64) or open3d-unofficial-arm (aarch64)
#   - System GL libraries (aarch64 only)
#
# Usage:
#   ./scripts/install_open3d.sh
#
# Safe to re-run (idempotent).

set -euo pipefail

say()  { printf "\033[1;34m[%s]\033[0m %s\n" "$1" "$2" >&2; }
ok()   { printf "\033[1;32m  \xe2\x9c\x94\033[0m %s\n" "$*" >&2; }
fail() { printf "\033[1;31m  \xe2\x9c\x98\033[0m %s\n" "$*" >&2; }

ARCH="$(uname -m)"

# ── [1/4] Check prerequisites ──────────────────────────────────────────────

say CHECK "Prerequisites"

PYTHON=$(command -v python3 || true)
if [ -z "$PYTHON" ]; then
  fail "python3 not found — install it first"
  exit 1
fi
PYVER=$("$PYTHON" --version 2>&1 | grep -oP '\d+\.\d+')
ok "Python $PYVER ($ARCH)"

PIP=$(command -v pip3 || true)
if [ -z "$PIP" ]; then
  say INSTALL "pip3 not found — installing python3-pip"
  sudo apt update && sudo apt install -y python3-pip
  PIP=$(command -v pip3)
fi
ok "pip3 found"

# ── [2/5] Install system dependencies ──────────────────────────────────────

say INSTALL "System dependencies"

if [ "$ARCH" = "aarch64" ]; then
  . /etc/os-release 2>/dev/null || true
  if echo "${ID:-}" | grep -qi debian; then
    GL_PKGS="libegl1 libgles2"
  else
    GL_PKGS="mesa-egl"
  fi
  sudo apt update
  sudo apt install -y --no-install-recommends \
    libgomp1 \
    $GL_PKGS || true
  ok "ARM64 GL libraries installed (${ID:-unknown})"
else
  ok "No extra system deps needed for $ARCH"
fi

# ── [3/5] Install Python dependencies (numpy, scipy, pyyaml) ───────────────

say INSTALL "Python dependencies"

for pkg in numpy scipy pyyaml; do
  if "$PYTHON" -c "import $pkg" 2>/dev/null; then
    ok "$pkg already installed"
  else
    $PIP install $PIP_OPTS "$pkg"
    ok "$pkg installed"
  fi
done

# ── [4/5] Install Open3D via pip ───────────────────────────────────────────

say INSTALL "Open3D"

PIP_OPTS="--break-system-packages"

if [ "$ARCH" = "aarch64" ]; then
  PKG="open3d-unofficial-arm"
else
  PKG="open3d"
fi

if "$PYTHON" -c "import open3d" 2>/dev/null; then
  ok "Open3D already installed"
else
  # shellcheck disable=SC2086
  $PIP install $PIP_OPTS "$PKG"
  ok "Open3D installed ($PKG)"
fi

# ── [5/5] Verify ────────────────────────────────────────────────────────────

say VERIFY "Verifying installation"

if "$PYTHON" -c "import open3d as o3d; print(o3d.__version__)" 2>&1; then
  VER=$("$PYTHON" -c "import open3d as o3d; print(o3d.__version__)")
  ok "Open3D version $VER"
else
  fail "Open3D import failed"
  exit 1
fi

echo ""
say DONE "Open3D installation complete on $ARCH"
