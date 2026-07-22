#!/usr/bin/env bash
# Record a running demo's screen (RViz) to a sped-up GIF for
# verification_artifacts/. Assumes the demo (gz sim + RViz) is already up and
# settled -- this script only captures and encodes, it does not launch
# anything, so it can point at any already-running X display.
#
# Usage: record_demo.sh <output.gif> [duration_s] [speedup] [display]
#   output.gif  path to write (e.g. verification_artifacts/demo_straight.gif)
#   duration_s  seconds of real time to capture (default 20)
#   speedup     playback speed multiplier, e.g. 2 or 3 (default 3)
#   display     X display to grab (default :99, the Xvfb headless display)
set -euo pipefail

OUT="${1:?usage: record_demo.sh <output.gif> [duration_s] [speedup] [display]}"
DURATION="${2:-20}"
SPEEDUP="${3:-3}"
DISPLAY_NUM="${4:-:99}"

WORKDIR="$(mktemp -d)"
trap 'rm -rf "${WORKDIR}"' EXIT

RAW="${WORKDIR}/raw.mp4"
PALETTE="${WORKDIR}/palette.png"

echo "[record_demo] capturing ${DURATION}s from display ${DISPLAY_NUM}..."
ffmpeg -y -f x11grab -video_size 1280x800 -framerate 10 -i "${DISPLAY_NUM}" \
  -t "${DURATION}" "${RAW}" -loglevel error

echo "[record_demo] encoding at ${SPEEDUP}x speed..."
FILTER="setpts=PTS/${SPEEDUP},fps=8,scale=800:-1:flags=lanczos"
ffmpeg -y -i "${RAW}" -vf "${FILTER},palettegen" "${PALETTE}" -loglevel error
ffmpeg -y -i "${RAW}" -i "${PALETTE}" \
  -filter_complex "${FILTER}[x];[x][1:v]paletteuse" \
  "${OUT}" -loglevel error

echo "[record_demo] wrote ${OUT}"
