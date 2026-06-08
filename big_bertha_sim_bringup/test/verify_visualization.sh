#!/usr/bin/env bash
# Headless-ish acceptance gate for the visualization module (issue #18).
#
# Asserts each RViz config under config/rviz/ loads in rviz2 without a fatal /
# plugin-load error, and that both PlotJuggler layouts under config/plotjuggler/
# are well-formed XML (python xml.etree). RViz needs a display (DISPLAY); the
# check times out rviz2 after a short load window and inspects its log -- a
# clean OpenGL init with no fatal/plugin error counts as a pass. Evidence ->
# verification_artifacts/visualization/.
echo "[verify] sourcing ROS 2 + workspace"
# Source ROS first (its setup scripts trip 'set -u'), then enable strict mode.
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -uo pipefail

ART="${ART:-verification_artifacts/visualization}"
RVIZ_SECS="${RVIZ_SECS:-12}"
mkdir -p "${ART}"
: > "${ART}/gate.txt"
PASS=0

CFG_RVIZ="big_bertha_sim_bringup/config/rviz"
CFG_PJ="big_bertha_sim_bringup/config/plotjuggler"

# --- PlotJuggler layouts: must parse as well-formed XML ---------------------
echo "[verify] PlotJuggler layouts parse (xml.etree)"
for layout in sensors control; do
  if python3 -c "import xml.etree.ElementTree as ET; ET.parse('${CFG_PJ}/${layout}.xml')" \
       2>>"${ART}/pj_parse.log"; then
    echo "[PASS] plotjuggler ${layout}.xml is well-formed" | tee -a "${ART}/gate.txt"
  else
    echo "[FAIL] plotjuggler ${layout}.xml parse error" | tee -a "${ART}/gate.txt"
    PASS=1
  fi
done

# --- RViz configs: must load without a fatal / plugin-load error ------------
if [ -z "${DISPLAY:-}" ]; then
  echo "[WARN] no DISPLAY; skipping rviz2 load checks (run on a display)" \
    | tee -a "${ART}/gate.txt"
else
  # Some desktop sessions (e.g. a snap-confined terminal) inject an
  # incompatible libpthread that breaks Qt apps; strip that influence.
  unset GTK_EXE_PREFIX GTK_PATH LOCPATH GIO_MODULE_DIR GDK_PIXBUF_MODULE_FILE \
        LD_PRELOAD 2>/dev/null || true
  LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' \
    | grep -v '/snap/' | paste -sd: -)"
  export LD_LIBRARY_PATH

  for cfg in simulation mapping planning integration; do
    echo "[verify] rviz2 -d ${cfg}.rviz"
    timeout "${RVIZ_SECS}" rviz2 -d "${CFG_RVIZ}/${cfg}.rviz" \
      > "${ART}/rviz_${cfg}.log" 2>&1
    EC=$?
    GL=$(grep -c 'OpenGl version' "${ART}/rviz_${cfg}.log")
    FATAL=$(grep -icE 'fatal|segmentation|core dumped|terminate called|what\(\):|does not exist|Failed to load|Could not load' "${ART}/rviz_${cfg}.log")
    # 124 == ran the full timeout window (loaded + stayed up); GL init present,
    # no fatal/plugin error -> pass.
    if [ "${EC}" = "124" ] && [ "${GL}" -gt 0 ] && [ "${FATAL}" -eq 0 ]; then
      echo "[PASS] ${cfg}.rviz loaded (GL init, no fatal)" | tee -a "${ART}/gate.txt"
    else
      echo "[FAIL] ${cfg}.rviz (exit=${EC} gl=${GL} fatal=${FATAL})" \
        | tee -a "${ART}/gate.txt"
      PASS=1
    fi
  done
fi

echo "================ GATE ================"; cat "${ART}/gate.txt"
[ "${PASS}" -eq 0 ] && echo "[verify] VISUALIZATION GATE: PASS" \
  || echo "[verify] VISUALIZATION GATE: FAIL"
exit "${PASS}"
