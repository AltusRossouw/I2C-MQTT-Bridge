async function fetchJSON(url) {
  const r = await fetch(url, { cache: "no-store" });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
}
async function postJSON(url, body) {
  const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json().catch(() => ({}));
}
function $(sel) { return document.querySelector(sel); }

function rgbToHex(r, g, b) {
  const h = (n) => Math.max(0, Math.min(255, n)).toString(16).padStart(2, "0");
  return `#${h(r)}${h(g)}${h(b)}`;
}

async function loadStatusAndRegisters() {
  try {
    const data = await fetchJSON("/api/registers");
    const st = data.status || {};
    $("#wifi").textContent = `WiFi: ${st.wifiConnected ? "Connected" : "Disconnected"}`;
    $("#ip").textContent = `IP: ${st.ip || "-"}`;
    $("#mqtt").textContent = `MQTT: ${st.mqttConnected ? "Connected" : "Disconnected"}`;

    // Signals
    $("#enled").textContent = (st.EN_LED_state ?? "-");
    $("#pwmDuty").textContent = (st.pwmDuty == null || st.pwmDuty < 0) ? "-" : `${st.pwmDuty}%`;
    $("#pwmHz").textContent = (st.pwmHz == null || st.pwmHz < 0) ? "-" : `${st.pwmHz} Hz`;

    // Registers list
    const regs = (data.registers || []).map(r => {
      const addr = `0x${(r.address || 0).toString(16).padStart(2, "0").toUpperCase()}`;
      return `<div class="reg"><code>${addr}</code> <span class="name">${r.name || ""}</span> <span class="val">${r.value}</span></div>`;
    }).join("");
    $("#registers").innerHTML = regs || "<p>No registers loaded.</p>";
  } catch (e) {
    console.error("Failed to load registers", e);
  }
}

async function loadRgb() {
  try {
    const rgb = await fetchJSON("/api/rgb");
    const hex = rgbToHex(rgb.r || 0, rgb.g || 0, rgb.b || 0);
    $("#rgbColor").value = hex;
  } catch (e) {
    console.error("Failed to load RGB", e);
  }
}

async function setRgb() {
  const color = $("#rgbColor").value;
  try {
    await postJSON("/api/rgb", { color });
  } catch (e) {
    console.error("Set RGB failed", e);
  }
}

function init() {
  $("#setRgb").addEventListener("click", setRgb);

  loadStatusAndRegisters();
  loadRgb();
  setInterval(loadStatusAndRegisters, 2000);

  // Live updates via WebSocket
  try {
    const ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onmessage = (ev) => {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === "EN_LED_state" && typeof msg.value === "number") {
          $("#enled").textContent = msg.value;
        } else if (msg.type === "pwm_input") {
          if (typeof msg.duty === "number") $("#pwmDuty").textContent = msg.duty < 0 ? "-" : `${msg.duty}%`;
          if (typeof msg.hz === "number") $("#pwmHz").textContent = msg.hz < 0 ? "-" : `${msg.hz} Hz`;
        } else if (msg.type === "rgb") {
          const hex = rgbToHex(msg.r || 0, msg.g || 0, msg.b || 0);
          $("#rgbColor").value = hex;
        }
      } catch {}
    };
  } catch {}
}

document.addEventListener("DOMContentLoaded", init);