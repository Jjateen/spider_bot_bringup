#!/usr/bin/env bash
# Start the iceoryx RouDi daemon for Cyclone DDS shared-memory data sharing.
# Run once per session, before any node using dds_env.sh starts publishing
# (Cyclone falls back to normal transport if RouDi isn't up, so order is not
# strict, but starting it first avoids the fallback window).
exec iox-roudi "$@"
