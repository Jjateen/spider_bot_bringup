#!/usr/bin/env bash
# Run the Big Bertha simulation stack in the prebuilt container. The image is a
# ROS 2 Jazzy workspace at /home/ros/spider_bot_bringup; this repo is mounted
# onto its src/. Build (colcon) and launch from inside the interactive shell.
#
#   ./docker_run.sh                                   # interactive shell
#   BB_DOCKER_IMG=big_bertha_sim:latest ./docker_run.sh
#   BB_DDS_CONF=/path/to/profile.xml ./docker_run.sh  # custom DDS profile
set -euo pipefail

IMG="${BB_DOCKER_IMG:-big_bertha_sim:cyclonedds}"

# DDS profile to mount at /etc/ros/cyclonedds.xml.
# A user-supplied profile wins; otherwise the script generates one for the
# HOST's actual interfaces (the container runs --network host). The board-side
# profile pins fixed names like wlan0 that do not exist on a laptop, so
# reusing it would leave the container invisible to external DDS participants.
if [ -n "${BB_DDS_CONF:-}" ]; then
  DDS_SRC="$BB_DDS_CONF"
elif [ -f "$PWD/cyclonedds_docker.xml" ]; then
  DDS_SRC="$PWD/cyclonedds_docker.xml"
else
  DDS_SRC="$(mktemp /tmp/cyclonedds_docker_XXXX.xml)"
  {
    cat <<'EOF'
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" priority="20" multicast="true"/>
EOF
    # Every up, multicast-capable LAN interface (wlan/eth/...): same-host
    # traffic wins on loopback, external machines are reached on the LAN.
    while read -r iface; do
      echo "        <NetworkInterface name=\"$iface\" priority=\"10\" multicast=\"true\"/>"
    done < <(ip -o link show 2>/dev/null |
      awk -F': ' '{gsub(/ /,"",$2); print $2}' |
      while read -r i; do
        flags="$(ip -o link show "$i" 2>/dev/null | awk -F'<|>' '{print $2}')"
        [ -n "$flags" ] || continue
        case "$flags" in *UP*MULTICAST*|*MULTICAST*UP*) ;; *) continue ;; esac
        case "$i" in lo|docker*|veth*|tailscale*|br-*) continue ;; esac
        ip -4 addr show "$i" 2>/dev/null | grep -q ' inet ' && echo "$i"
      done)
    # Tailscale carries no multicast; reach those peers by unicast only.
    # Require an IPv4 address: without one Cyclone cannot bind the interface
    # and logs a false "optional interface was not found" on every start.
    if ip -4 addr show tailscale0 2>/dev/null | grep -q ' inet '; then
      echo '        <NetworkInterface name="tailscale0" priority="1" multicast="false" presence_required="false"/>'
    fi
    cat <<'EOF'
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
      <MaxMessageSize>1200B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
    <Tracing>
      <Verbosity>severe</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
  } > "$DDS_SRC"
fi

[ -f "$DDS_SRC" ] || { echo "no DDS profile at $DDS_SRC" >&2; exit 1; }

# X11 GUI (rviz2): hand the container the host's MIT-MAGIC-COOKIE so X accepts
# it. With --network host the display sockets (path and abstract) are reachable,
# but without XAUTHORITY the container has no cookie and X replies
# "Authorization required, but no authorization protocol specified".
XAUTH_SRC="${XAUTHORITY:-$HOME/.Xauthority}"
XAUTH_DST="/tmp/.container-xauth"
if [ -f "$XAUTH_SRC" ]; then
  XAUTH_ARGS=(-v "$XAUTH_SRC:$XAUTH_DST:ro" -e "XAUTHORITY=$XAUTH_DST")
else
  XAUTH_ARGS=()
  echo "WARNING: no Xauthority cookie at $XAUTH_SRC; rviz2 will not open" >&2
fi

exec docker run -it --rm \
  --network host \
  --ipc host \
  --privileged \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/etc/ros/cyclonedds.xml \
  -v "$PWD:/home/ros/spider_bot_bringup/src" \
  -v "$DDS_SRC:/etc/ros/cyclonedds.xml:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  "${XAUTH_ARGS[@]}" \
  -w /home/ros/spider_bot_bringup \
  "$IMG" \
  "${@:-/bin/bash}"