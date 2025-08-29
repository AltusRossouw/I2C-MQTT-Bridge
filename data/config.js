function $(sel) { return document.querySelector(sel); }

async function fetchJSON(url) {
  const r = await fetch(url, { cache: "no-store" });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
}
async function fetchText(url) {
  const r = await fetch(url, { cache: "no-store" });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.text();
}
async function postText(url, body) {
  const r = await fetch(url, { method: "POST", headers: { "Content-Type": "text/plain" }, body });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.text();
}
async function postJSON(url, body) {
  const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json().catch(() => ({}));
}

async function loadConfig() {
  try {
    const cfg = await fetchJSON("/api/config");
    $("#ssid").value = cfg.ssid ?? "";
    $("#mqttHost").value = cfg.mqttHost ?? "";
    $("#mqttPort").value = cfg.mqttPort ?? 1883;
    $("#mqttUser").value = cfg.mqttUser ?? "";
    $("#mqttBaseTopic").value = cfg.mqttBaseTopic ?? "";

    $("#sdaPin").value = cfg.sdaPin ?? 21;
    $("#sclPin").value = cfg.sclPin ?? 22;
    $("#digitalPin").value = cfg.digitalPin ?? 27;
    $("#digitalPullup").checked = !!cfg.digitalPullup;
    $("#pwmPin").value = cfg.pwmPin ?? 26;

    // RGB block
    const rgb = cfg.rgb || {};
    $("#rgbEnable").checked = !!rgb.enable;
    $("#rgbRPin").value = rgb.rPin ?? 23;
    $("#rgbGPin").value = rgb.gPin ?? 32;
    $("#rgbBPin").value = rgb.bPin ?? 33;
    $("#rgbActiveLow").checked = !!rgb.activeLow;

    // status IP
    try {
      const regs = await fetchJSON("/api/registers");
      $("#ip").textContent = `IP: ${regs?.status?.ip || "-"}`;
    } catch {}
  } catch (e) {
    console.error("Failed to load config", e);
    alert("Failed to load config");
  }
}

async function loadRegistersTxt() {
  try {
    const txt = await fetchText("/registers.txt");
    $("#registersText").value = txt;
  } catch (e) {
    console.error("Failed to load registers.txt", e);
    alert("Failed to load registers.txt");
  }
}

async function saveRegistersTxt() {
  try {
    await postText("/registers.txt", $("#registersText").value);
    alert("registers.txt saved and reloaded.");
  } catch (e) {
    console.error("Save registers.txt failed", e);
    alert("Save registers.txt failed: " + e.message);
  }
}

async function saveConfigAndReboot() {
  const payload = {
    ssid: $("#ssid").value,
    mqttHost: $("#mqttHost").value,
    mqttPort: Number($("#mqttPort").value || 1883),
    mqttUser: $("#mqttUser").value,
    mqttBaseTopic: $("#mqttBaseTopic").value,
    sdaPin: Number($("#sdaPin").value),
    sclPin: Number($("#sclPin").value),
    digitalPin: Number($("#digitalPin").value),
    digitalPullup: $("#digitalPullup").checked,
    pwmPin: Number($("#pwmPin").value),
    rgb: {
      enable: $("#rgbEnable").checked,
      rPin: Number($("#rgbRPin").value),
      gPin: Number($("#rgbGPin").value),
      bPin: Number($("#rgbBPin").value),
      activeLow: $("#rgbActiveLow").checked
    }
  };
  // Only send passwords if non-empty
  const wifiPass = $("#wifipass").value;
  const mqttPass = $("#mqttPass").value;
  if (wifiPass && wifiPass.length) payload.wifipass = wifiPass;
  if (mqttPass && mqttPass.length) payload.mqttPass = mqttPass;

  try {
    await postJSON("/api/save-config", payload);
    alert("Saved. Device will reboot. This page will try to reload in ~5s.");
    setTimeout(() => location.reload(), 5000);
  } catch (e) {
    console.error("Save config failed", e);
    alert("Save config failed: " + e.message);
  }
}

function init() {
  $("#saveRegisters").addEventListener("click", saveRegistersTxt);
  $("#reloadRegisters").addEventListener("click", loadRegistersTxt);
  $("#saveConfig").addEventListener("click", saveConfigAndReboot);

  loadConfig();
  loadRegistersTxt();
}

document.addEventListener("DOMContentLoaded", init);

function rssiToBars(rssi) {
  // Simple 0..4 bars based on RSSI (dBm)
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  if (rssi >= -85) return 1;
  return 0;
}

function renderWifiScanResults(results) {
  const wrap = document.getElementById("wifiScanResults");
  if (!wrap) return;
  if (!Array.isArray(results)) results = [];

  // Deduplicate by SSID keeping strongest
  const bestBySsid = new Map();
  for (const ap of results) {
    const ssid = ap.ssid || "";
    if (!ssid) continue; // skip hidden for simple UI
    const prev = bestBySsid.get(ssid);
    if (!prev || (ap.rssi > prev.rssi)) bestBySsid.set(ssid, ap);
  }
  const list = Array.from(bestBySsid.values())
    .sort((a, b) => b.rssi - a.rssi);

  if (!list.length) {
    wrap.innerHTML = '<div class="hint">No networks found. Try again.</div>';
    return;
  }

  // Basic list of buttons to pick SSID
  wrap.innerHTML = list.map(ap => {
    const bars = "▁▂▃▄▅".slice(0, rssiToBars(ap.rssi) + 1); // visual bars 0..4
    const lock = ap.secureStr && ap.secureStr !== "OPEN" ? "🔒" : "🔓";
    const ch = ap.channel != null ? ` ch${ap.channel}` : "";
    return `
      <button type="button" class="wifi-ap-item" data-ssid="${ap.ssid.replace(/"/g, "&quot;")}">
        <span class="ssid">${ap.ssid}</span>
        <span class="meta">${lock}${ch} · ${ap.rssi} dBm</span>
        <span class="bars" aria-hidden="true">${bars}</span>
      </button>
    `;
  }).join("");

  // Wire clicks to fill the SSID input
  wrap.querySelectorAll(".wifi-ap-item").forEach(btn => {
    btn.addEventListener("click", () => {
      const ssid = btn.getAttribute("data-ssid") || "";
      const inp = document.getElementById("ssid");
      if (inp) {
        inp.value = ssid;
        inp.dispatchEvent(new Event("input", { bubbles: true }));
      }
      // Focus password next if it exists
      const pw = document.getElementById("wifipass");
      if (pw) pw.focus();
    });
  });
}

async function doWifiScan(scanBtn) {
  const btn = scanBtn || document.getElementById("scanWifiBtn");
  const wrap = document.getElementById("wifiScanResults");
  if (btn) {
    btn.disabled = true;
    btn.textContent = "Scanning…";
  }
  if (wrap) wrap.innerHTML = '<div class="hint">Scanning for networks…</div>';
  try {
    const results = await fetchJSON("/api/wifi/scan");
    renderWifiScanResults(results);
  } catch (e) {
    console.error("WiFi scan failed", e);
    if (wrap) wrap.innerHTML = '<div class="error">Scan failed. Please try again.</div>';
  } finally {
    if (btn) {
      btn.disabled = false;
      btn.textContent = "Scan Networks";
    }
  }
}

function injectWifiScanUI() {
  const ssidInput = document.getElementById("ssid");
  if (!ssidInput) return;

  const ssidLabel = ssidInput.closest("label") || ssidInput.parentElement || ssidInput;

  // Button: Scan Networks
  const scanBtn = document.createElement("button");
  scanBtn.type = "button";
  scanBtn.id = "scanWifiBtn";
  scanBtn.textContent = "Scan Networks";
  scanBtn.style.marginTop = "0.5rem";

  // Results container
  const resultsDiv = document.createElement("div");
  resultsDiv.id = "wifiScanResults";
  resultsDiv.style.display = "grid";
  resultsDiv.style.gap = "0.25rem";
  resultsDiv.style.marginTop = "0.5rem";

  // Minimal item styling without changing your existing stylesheet
  const style = document.createElement("style");
  style.textContent = `
    .wifi-ap-item {
      display: flex; justify-content: space-between; align-items: center;
      padding: 0.45rem 0.6rem; border: 1px solid #ddd; border-radius: 6px;
      background: #fff; cursor: pointer;
    }
    .wifi-ap-item .ssid { font-weight: 600; }
    .wifi-ap-item .meta { color: #666; font-size: 0.85em; margin-left: 0.75rem; }
    .wifi-ap-item .bars { font-family: monospace; color: #444; margin-left: 0.75rem; }
    .hint { color: #666; font-size: 0.9em; }
    .error { color: #b00; font-size: 0.9em; }
  `;

  // Insert into DOM after the SSID label
  ssidLabel.insertAdjacentElement("afterend", scanBtn);
  scanBtn.insertAdjacentElement("afterend", resultsDiv);
  document.head.appendChild(style);

  scanBtn.addEventListener("click", () => doWifiScan(scanBtn));
}

document.addEventListener("DOMContentLoaded", injectWifiScanUI);