# ESP32 NMEA Link

Access Point + captive portal that serves a **web UI** with two modes:

- **NMEA Monitor** (UART RX=16, category filters, UDP forward).
- **NMEA Generator** (UART TX=17 + UDP, up to **4 slots** with editable templates and **automatic checksum**).

Runs on **both cores**: networking/HTTP on Core 0, NMEA/LED on Core 1 for a smooth UI.

---

## 🛠️ Features

- **Wi-Fi AP**: `SSID: NMEA_Link`, `password: 12345678`, with **captive portal** (auto-opens the web UI).
- **Dark, mobile-friendly UI** (monospace).
- **Monitor**:
  - UART **RX=16** (baud options: 4800 / 9600 / 38400 / 115200).
  - Category filters (GPS, AIS, WEATHER, HEADING, SOUNDER, VELOCITY, RADAR, TRANSDUCER, OTHER).
  - **Start/Pause**, **Clear**, and polling speed (25/50/75/100%).
  - Valid frames forwarded via **UDP 10110** (broadcast).
- **Generator**:
  - UART **TX=17** + **UDP 10110**.
  - Up to **4 simultaneous slots**, each with:
    - **Sensor** and **sentence** (NMEA 0183) or **CUSTOM**.
    - **Editable template** (editor hides `*HH`; checksum is recalculated live).
    - **Per-slot interval**: 0.1 s / 0.5 s / 1 s / 2 s.
    - Slot enable/disable.
  - **Start/Pause**, **Clear output**, and **Back to NMEA Monitor** (full-width button).
- **LED states (NeoPixel GPIO 48)**:
  - Cyan: boot.
  - Green: valid RX.
  - Red: invalid RX.
  - Blue: TX from Generator.
- **Quiet logs**: only **boot information** is printed to the serial terminal (no frame spam, no UI events).

> Monitor and Generator do **not** run at the same time. Switching pages pauses the previous mode.

---

## 📡 Network / Access

1. Power the ESP32 and connect to **`NMEA_Link`** (password `12345678`).
2. The **captive portal** should open the UI; if not, go to `http://192.168.4.1/`.
3. mDNS: `http://nmeareader.local` (may vary by OS).

**UDP**: device emits on **10110** to the AP broadcast (x.x.x.255). Works with OpenPlotter/Signal K/other NMEA 0183 apps.

---

## 🔌 Pins / Hardware

- **UART RX** (Monitor): **GPIO 16**
- **UART TX** (Generator): **GPIO 17**
- **NeoPixel**: **GPIO 48** (1 LED)
- Tested with PlatformIO on **ESP32-S3-DevKitC-1**.

---

## 🚀 Quick start

### Monitor
1. Select **baudrate** (active one is highlighted).
2. Press **Start** to begin viewing incoming frames.
3. Use **category filters** and **Clear**.
4. Adjust **polling speed** if needed.

### Generator
1. For each **slot**: enable it, pick **sensor** and **sentence** (or *CUSTOM*).
2. Edit the **template** (without `*HH`); checksum updates automatically.
3. Set the slot’s **interval** (0.1/0.5/1/2 s).
4. Select **baudrate**, press **Start** to transmit.
5. **Back to NMEA Monitor** pauses the generator automatically.

---

## 📦 Build (PlatformIO)

Suggested `platformio.ini`:
```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
build_flags =
  -DARDUINO_USB_MODE=1
  -DARDUINO_USB_CDC_ON_BOOT=1
lib_deps =
  adafruit/Adafruit NeoPixel@^1.12.0

---

****************************************************************************************************************
****************************************************************************************************************

## 📑 Supported sentences (NMEA 0183)

GPS: GLL, RMC, VTG, GGA, GSA, GSV, DTM, ZDA

WEATHER: MWD, MWV, VWR, VWT, MTW

HEADING: HDG, HDT, HDM, THS, ROT, RSA

SOUNDER: DBT, DPT, DBK, DBS

VELOCITY: VHW, VLW, VBW

RADAR: TLL, TTM, TLB, OSD

TRANSDUCER: XDR

AIS: AIVDM, AIVDO

CUSTOM: free-form (editor with auto checksum)

****************************************************************************************************************
****************************************************************************************************************

## 🧭 Integration

OpenPlotter / Signal K / Nav software: listen to UDP 10110 on the Wi-Fi interface connected to NMEA_Link.

****************************************************************************************************************
****************************************************************************************************************

🔒 Notes / Limitations

UI is served over HTTP (not HTTPS) for simplicity on the ESP32.

Captive portal behavior depends on the client OS (may not always auto-open).

mDNS support varies by OS.

****************************************************************************************************************
****************************************************************************************************************

## 🗺️ Roadmap

Persist configuration (NVS).

Web OTA.

Export/Import templates.

Static assets (minified/gzip).

Unified language selector with more locales.

****************************************************************************************************************
****************************************************************************************************************

## ✨ Credits

Firmware & UI by Matías Scuppa — by Themys.

****************************************************************************************************************
****************************************************************************************************************

## 📝 License

MIT — use and modify freely; PRs welcome.


