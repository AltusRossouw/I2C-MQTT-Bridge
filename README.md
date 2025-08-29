# ESP32 I2C → MQTT Bridge

A generic, configurable bridge that periodically reads registers from an I2C device and publishes them to MQTT, while also accepting MQTT commands to write those registers back over I2C. It includes a built‑in web UI for configuration, a REST API, OTA updates, and optional RGB status LED feedback.

This project is intentionally device‑agnostic. You can use it with any 3.3V I2C peripheral by setting:
- SDA/SCL pins
- The I2C 7‑bit device address (called “I2C Device Address” or “start address”)
- A simple registers.txt mapping of register names to addresses and lengths

No vendor‑specific references or assumptions are embedded in the firmware.

---

## Features

- Generic I2C→MQTT translator for ESP32
- Configurable:
  - I2C SDA/SCL pins
  - I2C device address (0–127)
  - GPIO input pin to report a digital state (EN_LED_state)
  - PWM input pin to report duty (%) and frequency (Hz)
  - MQTT broker (host, port, credentials, base topic)
  - Optional RGB LED pins and polarity
- MQTT publishing (non‑retained) of:
  - Register values by I2C address (e.g. <base>0x30)
  - Digital input state (<base>EN_LED_state)
  - PWM input (<base>pwm_input_duty, <base>pwm_input_hz)
  - RGB state (<base>rgb, JSON)
  - Device IP (<base>ip)
  - A periodic state snapshot (<base>state, JSON)
- MQTT control topics to write I2C registers and set RGB color
- Web UI served from device for easy setup
- REST API for programmatic configuration and status
- OTA firmware updates (ArduinoOTA)
- Publishes only non‑retained messages by default to avoid stale broker state

---

## Hardware Requirements

- ESP32 development board
- 3.3V I2C device connected to ESP32 SDA/SCL with appropriate pull‑up resistors (some modules include them)
- Optional:
  - RGB LED connected to three PWM‑capable GPIOs (or an RGB module/strip that can be driven by LEDC PWM)
  - One digital input source for EN_LED_state signal
  - One PWM input source for measuring frequency and duty

Notes:
- ESP32 GPIO34–39 are input‑only and cannot drive LEDs.
- Ensure your I2C device voltage and logic levels are 3.3V compatible.

---

## Wiring (example)

- SDA → ESP32 GPIO 21 (configurable)
- SCL → ESP32 GPIO 22 (configurable)
- Optional digital input → e.g., GPIO 27 (configurable; can enable internal pull‑up)
- Optional PWM input → e.g., GPIO 26 (configurable)
- Optional RGB LED:
  - R → e.g., GPIO 23
  - G → e.g., GPIO 32
  - B → e.g., GPIO 33
  - Set “Active‑Low” if using common‑anode RGB

All pins are configurable in the web UI or via the API.

---

## Build and Flash

You can use Arduino IDE or PlatformIO. Ensure you also upload the data (web UI files) to the device’s LittleFS partition.

### Arduino IDE

1. Install ESP32 board support via Boards Manager.
2. Install libraries:
   - ESP Async WebServer
   - AsyncTCP (ESP32)
   - ArduinoJson
   - PubSubClient
   - ArduinoOTA (comes with ESP32 core)
   - LittleFS (ESP32) and the Arduino ESP32 LittleFS uploader plugin
3. Open the project, select “ESP32 Dev Module”.
4. Flash the sketch.
5. Use the LittleFS uploader to upload the contents of the `data/` folder (index.html, config.html, script.js, config.js, styles.css, registers.txt).

### PlatformIO (recommended)

1. Add required libs in your `platformio.ini` or use lib_deps (e.g., AsyncTCP, ESPAsyncWebServer, ArduinoJson, PubSubClient).
2. Build and upload firmware:
   - `pio run -t upload`
3. Upload LittleFS data:
   - `pio run -t uploadfs`

---

## First Boot and Access

- The device tries to connect to Wi‑Fi using stored credentials.
- If it cannot connect within ~15 seconds, it starts an open AP with a name like `RadarESP32-XXXX`. Connect to that AP and browse to `http://192.168.4.1/` to configure.
- Once connected to your Wi‑Fi, find the device IP from your router or mDNS (you can also read `<base>ip` on MQTT after it connects).

Open the configuration UI at:
- http://192.168.4.1/config.html

---

## Configuration

All configuration is stored in NVS/Preferences and survives reboots. You can change it via the web UI or the REST API.

- Wi‑Fi: SSID and password
- MQTT:
  - Host, Port, Username, Password
  - Base Topic (must end with “/”; if left empty, the device auto‑generates one like `radar/<CHIPID>/`)
- I2C:
  - SDA pin
  - SCL pin
  - I2C Device Address (0–127)
- EN_LED_state: Digital input pin and optional internal pull‑up
- PWM input: Pin to measure duty (%) and frequency (Hz)
- RGB LED: Enable, pins (R/G/B), and Active‑Low option

Saving config reboots the device to apply changes.

---

## I2C Register Mapping (registers.txt)

The device reads register definitions from `registers.txt` stored in LittleFS. You can view/edit it from the Config page.

Format: one entry per line
- name, 0xADDR, length
- length is 1..4 bytes (little‑endian on the wire)

Example:
```
# name, 0xADDR, length(1..4)
device.product_id,0x00,2
device.firmware_ver,0x02,2
sensor.threshold,0x30,2
sensor.on_time_ms,0x33,2
```

Behavior:
- The device polls these registers on a fixed interval.
- It publishes the value when it changes, and also on periodic refresh cycles.
- It also publishes all register values once when MQTT connects.

Tip: You can add or remove register definitions without reflashing. Save the file and the firmware reloads it automatically.

---

## MQTT Topics

Base topic: `<base>` (configurable; must end with “/”)

Published topics (non‑retained):
- `<base>0xNN` → decimal text value of the register at address 0xNN
- `<base>EN_LED_state` → “0” or “1”
- `<base>pwm_input_duty` → duty as integer percentage (or -1 if no signal)
- `<base>pwm_input_hz` → frequency in Hz (or -1 if no signal)
- `<base>rgb` → JSON: {"r":0..255,"g":0..255,"b":0..255}
- `<base>ip` → device IP string
- `<base>state` → JSON snapshot with:
  - ip, rssi, EN_LED_state, pwmDuty, pwmHz
  - gpio: sda, scl, digital, digitalPullup, pwm, rgb pins and activeLow
  - registers: array of {name, addr, len, value}

Control topics:
- Write a register by JSON:
  - Topic: `<base>set`
  - Payload: `{"reg":"0xNN","value":123}`  (reg can be “0xNN” or decimal; value accepts number or string like “0x1234”)
- Write a register by address‑scoped topic:
  - Topic: `<base>0xNN/set`
  - Payload: plain number like `123`, or JSON `{"value":123}`
- Set RGB color:
  - Topic: `<base>rgb/set`
  - Payload options:
    - `{"r":128,"g":64,"b":32}`
    - `{"hex":"#RRGGBB"}`
    - Plain `#RRGGBB` string

Notes:
- Messages published by the bridge are non‑retained to avoid stale state.
- If your broker already has retained values on these topics, clear them by publishing a retained null/empty payload once (using `-r -n` with mosquitto_pub).

Examples:
```
# Write register 0x30 to 200 via JSON set
mosquitto_pub -h <broker> -t "mybase/bridge/set" -m '{"reg":"0x30","value":200}'

# Write register 0x33 to 150 via address topic
mosquitto_pub -h <broker> -t "mybase/bridge/0x33/set" -m "150"

# Set RGB to magenta
mosquitto_pub -h <broker> -t "mybase/bridge/rgb/set" -m '{"hex":"#FF00FF"}'
```

---

## REST API

- GET `/api/config` → current configuration (secrets omitted or only updated when provided)
- POST `/api/save-config` → apply new configuration and reboot
  - Body: JSON object with any subset of:
    - ssid, wifipass
    - mqttHost, mqttPort, mqttUser, mqttPass, mqttBaseTopic
    - sdaPin, sclPin, digitalPin, digitalPullup, pwmPin
    - i2cAddr (0–127)
    - rgb: { enable, rPin, gPin, bPin, activeLow }
- GET `/api/registers` → JSON of registers + status
- GET `/api/rgb` → current RGB
- POST `/api/rgb` → set RGB; body supports:
  - `{"r":..,"g":..,"b":..}` and/or `{"color":"#RRGGBB"}`
- GET `/api/health` → `{"ok":true}`
- GET `/api/wifi/scan` → scan results (blocking call; the device preserves prior Wi‑Fi mode)

Files:
- GET `/registers.txt` → current file
- POST `/registers.txt` → upload new file (text/plain); auto‑reloads on save

---

## RGB Status Feedback (optional)

- While connecting to Wi‑Fi: red blink
- On successful Wi‑Fi connect: blue blink x3
- On periodic publish cycle: green pulse

You can disable RGB in the config or leave pins unassigned.

---

## OTA Updates

OTA is enabled after the device is connected to Wi‑Fi. Use ArduinoOTA tools to upload new firmware over the network. You can optionally set an OTA password in the code before building (not exposed in the UI).

---

## Security Considerations

- Use MQTT credentials on your broker.
- Consider isolating the device on a secure network segment.
- The web UI is served over HTTP on your LAN IP.
- By default, the firmware does not enable TLS for MQTT or HTTPS for the UI; if you need transport security, place the device behind a secure gateway or add TLS termination in your environment.

---

## Troubleshooting

- I2C not responding:
  - Verify the device’s 7‑bit address (configure it in the UI as “I2C Device Address”).
  - Check wiring, pull‑ups, and 3.3V logic compatibility.
  - Confirm SDA/SCL pins match your wiring.
- No MQTT messages:
  - Ensure broker host/port/user/pass are correct.
  - Verify base topic and subscribe to `#` under that base to see traffic.
- Registers not updating:
  - Confirm your registers.txt entries use the correct addresses and byte lengths.
  - Values publish on change and also on periodic refresh cycles (and once upon MQTT connect).
- RGB not lighting:
  - Ensure pins are output‑capable (not GPIO34–39).
  - Check Active‑Low setting for common‑anode LEDs.
- PWM shows -1:
  - This indicates no stable signal was detected during the sampling window; ensure your signal is present and within measurable range.

---

## Project Structure

- `src/main.cpp` — firmware
- `data/` — web UI and registers file stored on LittleFS
  - `index.html` — UI
  - `config.html` — configuration screen
  - `script.js`, `config.js`, `styles.css` — assets
  - `registers.txt` — register mapping file

Upload the `data/` folder to LittleFS after flashing the firmware.

---

## Acknowledgements

Built on top of:
- ESP32 Arduino core
- AsyncTCP and ESPAsyncWebServer
- ArduinoJson
- PubSubClient
- ArduinoOTA