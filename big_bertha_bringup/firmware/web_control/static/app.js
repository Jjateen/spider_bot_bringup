let config = null;
let angles = [];
let sending = false;

const statusBadge = document.getElementById("status-badge");
const statusText = document.getElementById("status-text");
const servoGrid = document.getElementById("servo-grid");
const previewBar = document.getElementById("preview-bar");
const btnSend = document.getElementById("btn-send");
const btnHome = document.getElementById("btn-home");

function setStatus(state, msg) {
  statusBadge.className = "badge " + state;
  statusText.textContent = msg;
}

function radToServoDeg(rad, idx) {
  return ((rad - config.policy_center[idx]) * 180 / Math.PI) * config.servo_direction[idx] + config.servo_offset[idx] + 90;
}

function fmtRad(rad) {
  return rad.toFixed(3);
}

function fmtDeg(deg) {
  return Math.round(deg) + "\u00b0";
}

async function fetchConfig() {
  const res = await fetch("/api/config");
  config = await res.json();
}

function buildPreview() {
  for (let i = 0; i < 12; i++) {
    const cell = document.createElement("div");
    cell.className = "preview-cell";
    cell.id = "preview-" + i;

    const name = document.createElement("span");
    name.className = "preview-name";
    name.textContent = config.joint_names[i].split(" ")[0];
    cell.appendChild(name);

    const radVal = document.createElement("span");
    radVal.className = "preview-rad";
    radVal.id = "preview-rad-" + i;
    radVal.textContent = fmtRad(config.initial_angles[i]);
    cell.appendChild(radVal);

    const degVal = document.createElement("span");
    degVal.className = "preview-deg";
    degVal.id = "preview-deg-" + i;
    degVal.textContent = fmtDeg(radToServoDeg(config.initial_angles[i], i));
    cell.appendChild(degVal);

    previewBar.appendChild(cell);
  }
}

function updatePreview() {
  for (let i = 0; i < 12; i++) {
    const radEl = document.getElementById("preview-rad-" + i);
    const degEl = document.getElementById("preview-deg-" + i);
    if (radEl) radEl.textContent = fmtRad(angles[i]);
    if (degEl) degEl.textContent = fmtDeg(radToServoDeg(angles[i], i));
  }
}

function buildUI() {
  for (let g = 0; g < config.leg_groups.length; g++) {
    const group = config.leg_groups[g];
    const card = document.createElement("div");
    card.className = "leg-card";
    card.dataset.leg = g;

    const title = document.createElement("h2");
    title.textContent = group.name;
    card.appendChild(title);

    for (let j = 0; j < group.indices.length; j++) {
      const idx = group.indices[j];
      const jointName = group.joints[j];
      const range = config.rad_ranges[idx];
      const initial = config.initial_angles[idx];

      const row = document.createElement("div");
      row.className = "servo-row";

      const label = document.createElement("span");
      label.className = "servo-label";
      label.textContent = jointName;
      row.appendChild(label);

      const input = document.createElement("input");
      input.type = "range";
      input.min = range[0];
      input.max = range[1];
      input.step = 0.001;
      input.value = initial;
      input.dataset.idx = idx;
      row.appendChild(input);

      const col = document.createElement("div");
      col.className = "servo-col";

      const radVal = document.createElement("span");
      radVal.className = "servo-rad";
      radVal.id = "sv-rad-" + idx;
      radVal.textContent = fmtRad(initial);
      col.appendChild(radVal);

      const degVal = document.createElement("span");
      degVal.className = "servo-degree";
      degVal.id = "sv-deg-" + idx;
      degVal.textContent = fmtDeg(radToServoDeg(initial, idx));
      col.appendChild(degVal);

      row.appendChild(col);
      card.appendChild(row);

      input.addEventListener("input", function () {
        const v = parseFloat(this.value);
        const i = parseInt(this.dataset.idx);
        document.getElementById("sv-rad-" + i).textContent = fmtRad(v);
        document.getElementById("sv-deg-" + i).textContent = fmtDeg(radToServoDeg(v, i));
        angles[i] = v;
        updatePreview();
      });

      angles[idx] = initial;
    }

    servoGrid.appendChild(card);
  }
}

async function sendAngles() {
  if (sending) return;
  sending = true;
  btnSend.disabled = true;
  setStatus("sending", "Sending...");

  try {
    const res = await fetch("/api/servos", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ angles: angles }),
    });
    const data = await res.json();
    if (data.ok) {
      setStatus("ok", "Sent (rad): [" + angles.map(fmtRad).join(", ") + "]");
      setTimeout(() => setStatus("idle", "Ready"), 4000);
    } else {
      setStatus("error", "Error: " + (data.error || "unknown"));
    }
  } catch (err) {
    setStatus("error", "Connection failed: " + err.message);
  } finally {
    sending = false;
    btnSend.disabled = false;
  }
}

async function homeAll() {
  if (sending) return;

  for (let i = 0; i < 12; i++) {
    angles[i] = config.policy_center[i];
    const input = document.querySelector(`input[data-idx="${i}"]`);
    if (input) {
      input.value = config.policy_center[i];
      document.getElementById("sv-rad-" + i).textContent = fmtRad(config.policy_center[i]);
      document.getElementById("sv-deg-" + i).textContent = fmtDeg(radToServoDeg(config.policy_center[i], i));
    }
  }
  updatePreview();

  setStatus("sending", "Homing...");
  try {
    const res = await fetch("/api/home", { method: "POST" });
    const data = await res.json();
    if (data.ok) {
      setStatus("ok", "Homed to policy center");
      setTimeout(() => setStatus("idle", "Ready"), 3000);
    } else {
      setStatus("error", "Home error: " + (data.error || "unknown"));
    }
  } catch (err) {
    setStatus("error", "Home failed: " + err.message);
  }
}

async function init() {
  await fetchConfig();
  buildPreview();
  buildUI();
  btnSend.addEventListener("click", sendAngles);
  btnHome.addEventListener("click", homeAll);
}

init();
