document.addEventListener('DOMContentLoaded', () => {
  const $ = id => document.getElementById(id);

  async function loadConfig() {
    const res = await fetch('/api/config');
    const cfg = await res.json();

    // Wi‑Fi
    $('ssid').value = cfg.ssid || '';

    // MQTT
    $('mqttHost').value = cfg.mqttHost || '';
    $('mqttPort').value = cfg.mqttPort || 1883;
    $('mqttUser').value = cfg.mqttUser || '';
    // mqttPass is intentionally not returned; leave field blank to keep unchanged
    $('mqttBaseTopic').value = cfg.mqttBaseTopic || '';

    // GPIO
    $('sdaPin').value = cfg.sdaPin ?? 21;
    $('sclPin').value = cfg.sclPin ?? 22;
    $('digitalPin').value = cfg.digitalPin ?? 27;
    $('digitalPullup').checked = !!cfg.digitalPullup;
    $('pwmPin').value = cfg.pwmPin ?? 26;

    // I2C device address
    const i2cAddr = (cfg.i2cAddr !== undefined) ? cfg.i2cAddr : 0x56;
    $('i2cAddr').value = '0x' + (i2cAddr & 0x7F).toString(16).padStart(2, '0');

    // RGB
    const rgb = cfg.rgb || {};
    $('rgbEnable').checked = !!rgb.enable;
    $('rgbRPin').value = rgb.rPin ?? 23;
    $('rgbGPin').value = rgb.gPin ?? 32;
    $('rgbBPin').value = rgb.bPin ?? 33;
    $('rgbActiveLow').checked = !!rgb.activeLow;
  }

  async function loadRegisters() {
    const res = await fetch('/registers.txt', { cache: 'no-cache' });
    const txt = await res.text();
    $('registersText').value = txt;
  }

  $('reloadRegisters').addEventListener('click', loadRegisters);
  $('saveRegisters').addEventListener('click', async () => {
    const body = $('registersText').value;
    const res = await fetch('/registers.txt', {
      method: 'POST',
      headers: { 'Content-Type': 'text/plain' },
      body
    });
    if (!res.ok) {
      alert('Failed to save registers.txt');
    } else {
      alert('Saved registers.txt');
    }
  });

  $('saveConfig').addEventListener('click', async () => {
    // Collect form fields
    const body = {};

    // Wi‑Fi
    body.ssid = $('ssid').value.trim();
    const wpass = $('wifipass').value; // blank means unchanged
    if (wpass && wpass.length) body.wifipass = wpass;

    // MQTT
    body.mqttHost = $('mqttHost').value.trim();
    body.mqttPort = parseInt($('mqttPort').value, 10) || 1883;
    body.mqttUser = $('mqttUser').value.trim();
    const mpass = $('mqttPass').value; // blank means unchanged
    if (mpass && mpass.length) body.mqttPass = mpass;
    body.mqttBaseTopic = $('mqttBaseTopic').value.trim();

    // GPIO
    body.sdaPin = parseInt($('sdaPin').value, 10);
    body.sclPin = parseInt($('sclPin').value, 10);
    body.digitalPin = parseInt($('digitalPin').value, 10);
    body.digitalPullup = !!$('digitalPullup').checked;
    body.pwmPin = parseInt($('pwmPin').value, 10);

    // I2C device address (accept hex "0xNN" or decimal)
    let aTxt = $('i2cAddr').value.trim();
    let addr = 0x56;
    if (aTxt.toLowerCase().startsWith('0x')) {
      addr = parseInt(aTxt, 16);
    } else {
      addr = parseInt(aTxt, 10);
    }
    if (!Number.isFinite(addr) || isNaN(addr)) addr = 0x56;
    addr = Math.max(0, Math.min(127, addr | 0));
    body.i2cAddr = addr;

    // RGB
    body.rgb = {
      enable: !!$('rgbEnable').checked,
      rPin: parseInt($('rgbRPin').value, 10),
      gPin: parseInt($('rgbGPin').value, 10),
      bPin: parseInt($('rgbBPin').value, 10),
      activeLow: !!$('rgbActiveLow').checked
    };

    const res = await fetch('/api/save-config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    if (res.ok) {
      alert('Saved. Device will reboot.');
    } else {
      alert('Save failed.');
    }
  });

  // Initial loads
  loadConfig().catch(console.error);
  loadRegisters().catch(console.error);
});document.addEventListener('DOMContentLoaded', () => {
  const $ = id => document.getElementById(id);

  async function loadConfig() {
    const res = await fetch('/api/config');
    const cfg = await res.json();

    // Wi‑Fi
    $('ssid').value = cfg.ssid || '';

    // MQTT
    $('mqttHost').value = cfg.mqttHost || '';
    $('mqttPort').value = cfg.mqttPort || 1883;
    $('mqttUser').value = cfg.mqttUser || '';
    // mqttPass is intentionally not returned; leave field blank to keep unchanged
    $('mqttBaseTopic').value = cfg.mqttBaseTopic || '';

    // GPIO
    $('sdaPin').value = cfg.sdaPin ?? 21;
    $('sclPin').value = cfg.sclPin ?? 22;
    $('digitalPin').value = cfg.digitalPin ?? 27;
    $('digitalPullup').checked = !!cfg.digitalPullup;
    $('pwmPin').value = cfg.pwmPin ?? 26;

    // I2C device address
    const i2cAddr = (cfg.i2cAddr !== undefined) ? cfg.i2cAddr : 0x56;
    $('i2cAddr').value = '0x' + (i2cAddr & 0x7F).toString(16).padStart(2, '0');

    // RGB
    const rgb = cfg.rgb || {};
    $('rgbEnable').checked = !!rgb.enable;
    $('rgbRPin').value = rgb.rPin ?? 23;
    $('rgbGPin').value = rgb.gPin ?? 32;
    $('rgbBPin').value = rgb.bPin ?? 33;
    $('rgbActiveLow').checked = !!rgb.activeLow;
  }

  async function loadRegisters() {
    const res = await fetch('/registers.txt', { cache: 'no-cache' });
    const txt = await res.text();
    $('registersText').value = txt;
  }

  $('reloadRegisters').addEventListener('click', loadRegisters);
  $('saveRegisters').addEventListener('click', async () => {
    const body = $('registersText').value;
    const res = await fetch('/registers.txt', {
      method: 'POST',
      headers: { 'Content-Type': 'text/plain' },
      body
    });
    if (!res.ok) {
      alert('Failed to save registers.txt');
    } else {
      alert('Saved registers.txt');
    }
  });

  $('saveConfig').addEventListener('click', async () => {
    // Collect form fields
    const body = {};

    // Wi‑Fi
    body.ssid = $('ssid').value.trim();
    const wpass = $('wifipass').value; // blank means unchanged
    if (wpass && wpass.length) body.wifipass = wpass;

    // MQTT
    body.mqttHost = $('mqttHost').value.trim();
    body.mqttPort = parseInt($('mqttPort').value, 10) || 1883;
    body.mqttUser = $('mqttUser').value.trim();
    const mpass = $('mqttPass').value; // blank means unchanged
    if (mpass && mpass.length) body.mqttPass = mpass;
    body.mqttBaseTopic = $('mqttBaseTopic').value.trim();

    // GPIO
    body.sdaPin = parseInt($('sdaPin').value, 10);
    body.sclPin = parseInt($('sclPin').value, 10);
    body.digitalPin = parseInt($('digitalPin').value, 10);
    body.digitalPullup = !!$('digitalPullup').checked;
    body.pwmPin = parseInt($('pwmPin').value, 10);

    // I2C device address (accept hex "0xNN" or decimal)
    let aTxt = $('i2cAddr').value.trim();
    let addr = 0x56;
    if (aTxt.toLowerCase().startsWith('0x')) {
      addr = parseInt(aTxt, 16);
    } else {
      addr = parseInt(aTxt, 10);
    }
    if (!Number.isFinite(addr) || isNaN(addr)) addr = 0x56;
    addr = Math.max(0, Math.min(127, addr | 0));
    body.i2cAddr = addr;

    // RGB
    body.rgb = {
      enable: !!$('rgbEnable').checked,
      rPin: parseInt($('rgbRPin').value, 10),
      gPin: parseInt($('rgbGPin').value, 10),
      bPin: parseInt($('rgbBPin').value, 10),
      activeLow: !!$('rgbActiveLow').checked
    };

    const res = await fetch('/api/save-config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    if (res.ok) {
      alert('Saved. Device will reboot.');
    } else {
      alert('Save failed.');
    }
  });

  // Initial loads
  loadConfig().catch(console.error);
  loadRegisters().catch(console.error);
});