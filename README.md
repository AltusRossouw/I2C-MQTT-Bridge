# ESP32 Azoteq Sensor Monitor (Web UI + MQTT + OTA + RGB)

An ESP32 firmware that:
- Reads sensor registers over I2C (default address 0x56).
- Publishes register values to MQTT by register address.
- Serves a local web dashboard (LittleFS) to view live data.
- Exposes a Config page to edit Wi‑Fi, MQTT, I2C pins (SDA/SCL), EN_LED input pin, PWM input pin, RGB LED pins, and registers.txt.
- Supports OTA updates via PlatformIO (ArduinoOTA).
- Publishes device IP to MQTT every 10 seconds.
- Publishes EN_LED_state (digital input) and PWM duty/frequency to MQTT and the dashboard.
- Drives an RGB LED (LEDC PWM) and provides status blinks:
  - Red blink while connecting to Wi‑Fi
  - Blue blink 3 times after successful Wi‑Fi connect
  - Green blink each time a full publish cycle completes (all registers published)

## Features

- I2C register reading/writing:
  - Read a configurable list of registers from `registers.txt` in LittleFS.
  - Each register can be 1–4 bytes (little‑endian).
  - Publish values under an MQTT base topic using register address suffixes (e.g. `.../0x36`).
  - Write registers via MQTT (`.../set` or `.../0xNN/set`).

- Digital input monitoring (EN_LED_state):
  - Configurable GPIO with optional internal pull‑up.
  - Publishes retained MQTT topic `<base>EN_LED_state` (1 = HIGH, 0 = LOW).
  - Visible on dashboard and streamed live via WebSocket.

- PWM input measuring:
  - Configurable PWM input pin.
  - Samples duty (%) and frequency (Hz) periodically.
  - Publishes retained MQTT topics `<base>pwm_input_duty` and `<base>pwm_input_hz`.
  - Visible on dashboard and streamed live via WebSocket.

- RGB LED output:
  - LEDC PWM on 3 channels, configurable pins and polarity (active‑low for common‑anode).
  - MQTT control at `<base>rgb/set` with `{"color":"#RRGGBB"}` or `{"r":..,"g":..,"b":..}`.
  - State published as retained JSON at `<base>rgb`.
  - Status animations: red while connecting, blue x3 on connect, green pulse on each full publish.

- Device IP publishing:
  - Retained MQTT topic `<base>ip` every 10 seconds.

- Web UI (LittleFS):
  - Dashboard: live EN_LED_state, PWM stats, RGB color, register list, connection status.
  - Config:
    - Wi‑Fi SSID/password with “Scan Networks” button and list of nearby APs.
    - MQTT: host, port, user, password, base topic.
    - Pins: SDA, SCL, EN_LED (pull‑up optional), PWM input.
    - RGB: enable, R/G/B pins, active‑low selection.
    - Inline `registers.txt` editor (changes applied immediately).

- OTA firmware updates:
  - ArduinoOTA built in; PlatformIO `espota` upload supported.

## Build

- Install PlatformIO.
- Select `esp32dev` environment.

Commands:
```
pio run                  # build
pio run -t uploadfs      # upload LittleFS web assets
pio run -t upload        # flash firmware over serial
pio device monitor       # serial monitor
```

OTA:
```
pio run -e esp32dev-ota -t upload --upload-port radar-XXXXXX.local
```

## Configuration

All config persisted in NVS (Preferences); edited from the web Config page:
- Wi‑Fi: SSID, password (if blank, unchanged). Use “Scan Networks” to pick SSID.
- MQTT: host, port, user, password (if blank, unchanged), base topic.
- Pins: SDA, SCL, EN_LED (pull‑up optional), PWM input.
- RGB: enable, R/G/B pins, active‑low selection.
- `registers.txt`: inline editor; changes applied immediately.

Default MQTT base topic: `radar/<CHIPID>/`

Notes:
- GPIO34–39 are input‑only; don’t assign them to RGB outputs.

## registers.txt format

CSV with three columns per line:
```
name,0xADDR,length
```
Example:
```
# name, 0xADDR, length(1..4)
app_info.product,0x00,2
config.tuning.energy_threshold_offset_default,0x30,2
config.tuning.on_duration_default,0x33,2
config.tuning.als_turnOn_threshold_default,0x36,2
```

## REST API

- GET `/api/registers` → {
  registers: [{ name, address, length, value }],
  status: { wifiConnected, ip, mqttConnected, EN_LED_state, pwmDuty, pwmHz, sda, scl, digitalPin, digitalPullup, pwmPin }
}
- GET `/api/config` → current configuration (secrets omitted)
- POST `/api/save-config` → apply config and reboot
- GET `/registers.txt` → raw file
- POST `/registers.txt` → replace file
- GET `/api/rgb` → current RGB (r,g,b, enabled, activeLow)
- POST `/api/rgb` → set RGB: `{"color":"#RRGGBB"}` or `{"r":..,"g":..,"b":..}`
- GET `/api/wifi/scan` → array of nearby networks:
```
[
  {
    "ssid": "MyWiFi",
    "rssi": -48,
    "bssid": "AA:BB:CC:DD:EE:FF",
    "channel": 6,
    "secure": 4,
    "secureStr": "WPA2-PSK",
    "hidden": false
  },
  ...
]
```

## MQTT Topics

Assume base `<base>` (e.g., `radar/ABCDEF/`).

Published (retained):
- `<base>0xNN` → register value (decimal string)
- `<base>EN_LED_state` → "0" or "1"
- `<base>pwm_input_duty` → percent as integer, or "-1" if no signal
- `<base>pwm_input_hz` → Hz as integer, or "-1" if no signal
- `<base>rgb` → `{"r":..,"g":..,"b":..}`
- `<base>ip` → device IP
- `<base>state` → JSON snapshot of device state and last-known register values

Control:
- `<base>set` with `{"reg":"0x36","value":10}`
- `<base>0x36/set` with `{"value":10}` or plain `10`
- `<base>rgb/set` with `{"color":"#RRGGBB"}` or `{"r":..,"g":..,"b":..}`

## Status LED Behaviors

- Wi‑Fi connecting: red blink until connected.
- Wi‑Fi connected: blue blink 3 times.
- Full publish cycle (all registers + state sent): green pulse.

## Notes

- Upload LittleFS before first flash to serve the web UI.
- Set an OTA password if deploying beyond a trusted network.