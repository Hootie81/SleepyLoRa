# SleepyLoRaMaster

SleepyLoRaMaster is the firmware for the Master actuator device
  The Master actuator device is a plantation shutter blind actuator with a 
  Heltec Wireless Stick Lite V3, voltage booster, motor driver, position sensor and RS485 module.
  Powered from a single cell lipo battery or USB.
  Can supply power to and control multiple slave devices via RS485.
  Exposes a web portal for configuration and firmware updates. (via button press or mqtt command)

## Project Structure
- `src/` — Main source code for the master controller
- `include/` — Shared headers and protocol definitions (e.g., Command_Register.h)
- `lib/` — External and custom libraries
- `platformio.ini` — PlatformIO project configuration

## Getting Started
1. Clone this repository or copy the project to your PlatformIO workspace.
2. Open with VS Code and PlatformIO extension.
3. Connect your master controller hardware (e.g., ESP32 or compatible MCU).
4. Configure any hardware-specific settings in `platformio.ini` or source files as needed.
5. Build and upload the firmware using PlatformIO.

## Communication Protocol
- **RS485 UART**: Used to communicate with slave devices. Each slave has a unique address.
- **LoRa**: Used for long-range wireless communication with a gateway using a custom encrypted protocol.


## License
SPDX-License-Identifier: MIT

Copyright (c) 2025 Chris Huitema

---
For more information, see the source code and comments in `src/main.cpp` and related files.
