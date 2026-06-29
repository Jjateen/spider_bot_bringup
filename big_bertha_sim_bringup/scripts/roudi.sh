#!/usr/bin/env bash
# Start the iceoryx RouDi daemon for Cyclone DDS shared-memory data sharing.
# demo_straight.launch.py starts this itself when dds_shm:=true (the
# default) -- you normally don't need to run this manually. Only needed for
# standalone tools/tests against an SHM-enabled CYCLONEDDS_URI outside the
# launch tree (e.g. scripts/lat_bench).
#
# Must be running BEFORE any SharedMemory Enable=true process starts:
# Cyclone does NOT fall back gracefully if RouDi isn't up -- it aborts the
# process at domain creation (confirmed via core dump, iox::errorHandler ->
# std::terminate). If RouDi is already running from a previous session,
# this exits immediately (fixed IPC channel name) -- that's fine, reuse it.
exec iox-roudi "$@"
