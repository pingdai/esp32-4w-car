# ESP32-S3 4WD Wi-Fi Smart Car

<p align="center">
  <b><a href="README.md">English</a></b> | <b><a href="README_zh.md">简体中文</a></b>
</p>

An **ESP32-S3** powered four-wheel differential drive (4WD) Wi-Fi remote-controlled smart car. No mobile app installation is required—the car broadcasts a SoftAP hotspot on boot, allowing any smartphone or PC browser to pilot the vehicle in real time using a low-latency virtual joystick. Features an on-board ST7789 IPS display dashboard, HC-SR04 ultrasonic forward collision avoidance, and fail-safe timeout protection.

---

## 🌟 Key Features

- **Zero-Install Web Remote**: Broadcasts its own SoftAP with an embedded HTTP and WebSocket server. Connect Wi-Fi, open the browser, and drive.
- **Omnidirectional Differential Joystick**: Dual-axis virtual joystick on the webpage smoothly maps X/Y offsets to differential motor speeds, supporting in-place zero-radius turns and curved steering.
- **Ultrasonic Collision Avoidance**: HC-SR04 measures distance at 100ms intervals. Forward drive is automatically suppressed when distance < 20cm, while reversing and steering remain available.
- **Real-Time IPS Dashboard**: 1.54" 240×240 ST7789 display provides live readouts for Wi-Fi SSID, IP, client connection status, ultrasonic obstacle distance, and motion states.
- **Fail-Safe Mechanism**:
  - **Disconnect Brake**: Motors stop immediately when the WebSocket connection closes.
  - **Heartbeat Timeout**: Automatically halts motors if no joystick packet is received within 500ms.
- **Pure Native ESP-IDF**: Built purely on native ESP-IDF v6.1 components (`esp_lcd`, `esp_http_server`, `ledc`, `esp_wifi`), free of heavy external dependencies.

---

## 🛠 Hardware Bill of Materials (BOM)

| Component | Model / Specs | Qty | Function |
|-----------|---------------|-----|----------|
| Mainboard | ESP32-S3-N16R8 | 1 | Computing, Wi-Fi AP, WebSocket, display, PWM control |
| Motor Driver | TB6612FNG Module | 1 | Dual H-bridge DC motor driver |
| Gear Motors | TT Motors + Rubber Wheels | 4 | 4WD differential drive (wired in parallel per side) |
| Chassis | 4WD Acrylic / Metal Chassis | 1 set | Mechanical structure |
| Ultrasonic Sensor | HC-SR04 | 1 | Forward obstacle detection and ranging |
| Display | 1.54" 240×240 IPS (ST7789, SPI) | 1 | Real-time status display |
| Motor Battery | 6V ~ 8.4V battery pack (e.g., 2S Li-ion) | 1 | Supplies motor VM (common GND with ESP32) |
| MCU Power | Power bank or regulated 5V source | 1 | Supplies ESP32-S3 via Type-C or 5V pin |

---

## 🔌 Hardware Wiring Guide

### 1. GPIO Pinout Table

| Signal | Peripheral | ESP32-S3 GPIO | Note |
|--------|------------|---------------|------|
| **PWMA** | TB6612 | **GPIO 1** | Left motor speed PWM (LEDC 20kHz) |
| **AIN1** | TB6612 | **GPIO 2** | Left motor direction 1 |
| **AIN2** | TB6612 | **GPIO 3** | Left motor direction 2 |
| **PWMB** | TB6612 | **GPIO 4** | Right motor speed PWM (LEDC 20kHz) |
| **BIN1** | TB6612 | **GPIO 5** | Right motor direction 1 |
| **BIN2** | TB6612 | **GPIO 6** | Right motor direction 2 |
| **STBY** | TB6612 | **GPIO 45** | Standby pin (pulled HIGH to enable) |
| **SCL / SCK** | ST7789 LCD | **GPIO 12** | SPI Clock |
| **SDA / MOSI** | ST7789 LCD | **GPIO 11** | SPI Data |
| **CS** | ST7789 LCD | **GPIO 10** | SPI Chip Select |
| **DC** | ST7789 LCD | **GPIO 9** | Data / Command select |
| **RST** | ST7789 LCD | **GPIO 8** | LCD Reset |
| **BLK** | ST7789 LCD | **GPIO 7** | Backlight control (HIGH = ON) |
| **TRIG** | HC-SR04 | **GPIO 46** | 10μs trigger pulse |
| **ECHO** | HC-SR04 | **GPIO 14** | Echo pulse (**Requires voltage divider!**) |

> ⚠️ **Critical Wiring Precautions**:
> 1. **ECHO Voltage Divider**: HC-SR04 operates at 5V and outputs 5V on ECHO, while ESP32-S3 GPIO pins tolerate up to 3.3V. You **must** divide the voltage (e.g., ECHO → 1kΩ → GPIO 14, and GPIO 14 → 2kΩ → GND).
> 2. **Common Ground (GND)**: The motor battery negative (-) terminal must share a common ground with ESP32-S3 GND. **Never** connect the motor battery directly to the ESP32 5V or 3.3V power rails.
> 3. **Avoid Strapping Pins**: GPIO 0 was avoided because pulling it down through the divider causes the chip to enter bootloader/download mode during reset. Firmware uses **GPIO 14** instead.

---

## 🚀 Build & Flash

A helper script `build.sh` is provided in the repository, targeting **ESP-IDF v6.1**.

### 1. Build Firmware

```bash
./build.sh
```

### 2. Flash to Device

Connect ESP32-S3 to your computer via USB (auto-detects `/dev/cu.usbmodem*` on macOS):

```bash
./build.sh flash
```

*Or specify the port manually:*
```bash
./build.sh flash /dev/cu.usbmodem1101
```

### 3. Monitor Serial Output

```bash
./build.sh monitor
```

---

## 🎮 How to Operate

1. **Power On**: On boot, the LCD lights up and initializes the SoftAP:
   - SSID: `4WD-Car`
   - Password: `12345678`
   - Default IP: `192.168.4.1`
2. **Connect Wi-Fi**: Connect your phone or laptop to `4WD-Car` using password `12345678`.
3. **Open Controller**: Open your browser and navigate to:
   ```
   http://192.168.4.1
   ```
4. **Drive**:
   - Drag the virtual joystick on the screen to drive forward, backward, or turn.
   - Release the joystick to center and brake automatically.
   - If an obstacle appears within 20cm, forward throttle is suppressed, and warning banners turn red on both the display and webpage. You can safely reverse or steer away.

---

## 📂 Project Structure

```
4w-car/
├── docs/
│   └── design.md          # Comprehensive architecture & wiring design document
├── main/
│   ├── CMakeLists.txt     # Main component build configuration
│   ├── index.html         # Web joystick interface (embedded into binary)
│   └── main.c             # Firmware logic (Wi-Fi/HTTP/WS/LCD/Motor/Ultrasonic)
├── CMakeLists.txt         # ESP-IDF top-level CMake configuration
├── sdkconfig.defaults     # Project defaults (target esp32s3, 16MB flash, WS support)
├── build.sh               # One-stop build, flash, and monitor script
├── .gitignore             # Git ignore rules
├── README.md              # English documentation
└── README_zh.md           # Chinese documentation
```

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).