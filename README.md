# 🚀 Pump Controller ESP32
## Web-Controlled Smart Pump System

<div align="center">
  <img src="./.gitassets/index.jpg" alt="Main page" width="300">
  <img src="./.gitassets/setup.jpg" alt="Setup page" width="300">
  <br>
  <sub>⚙️ Main Dashboard &nbsp;&nbsp;|&nbsp;&nbsp; 🛠️ Setup Wizard</sub>
</div>

---


## ✨ Features
- 📡 Web-based pressure monitoring
- 🔧 OTA-ready & configurable via browser
- 💡 LED status indicators
- 🔁 Auto-reset & fail-safe AP mode
- 🧠 Non-volatile memory for settings

---


## 🛠️ Build & Flash
> 💡 *Requires ESP-IDF framework and `just` task runner*

```bash
PORT="/dev/ttyUSB0" \
ESP_IDF="~/playground/esp-idf/"
just build flash
```

---


## 🌐First-Time Setup
On **first boot**, the device creates a captive Wi-Fi access point:

- **SSID:** `pumpcontroller`
- **IP:** `192.168.4.1`
- **Port:** `80`

### Steps:
1. Connect to `pumpcontroller` Wi-Fi
2. Open browser → `http://192.168.4.1`
3. Enter your **home Wi-Fi SSID + password**
4. Device reboots → joins your network
5. Find its IP in your router's DHCP list

✅ That IP becomes your **pump control dashboard** 🎯

> ⚠️ If you enter wrong credentials, the device will **not** connect.
> You’ll need to **factory reset** and try again.

---


## 🔄 Factory Reset
1. **Short pin `D15` to GND**
2. **Power on** the device
3. LED blinks **3 times** 🔴🔴🔴
4. **Remove** the short after the 3rd blink
5. Device reboots → access point mode restored
