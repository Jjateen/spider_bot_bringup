#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

xhost +local:
docker compose up -d
docker exec -it big_bertha_sim bash