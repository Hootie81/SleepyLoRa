# SleepyLoraSlave

SleepyLoraSlave is the firmware for the slave blind actuator in the SleepyLora Blinds System. It receives commands from the master controller via RS485 UART, controls a blind motor, and reports its position and state back to the master. The slave can be configured and updated via a built-in web portal.

## Features
- Receives commands from the master controller over RS485 UART
- Controls a blind motor and reports position/state
- Web portal for configuration (slave address, etc.)
- OTA firmware updates via web portal
- Powered by the master device
- Low-power operation

## Project Structure
- `src/` — Main source code for the slave controller
- `include/` — Shared headers and protocol definitions
- `lib/` — External and custom libraries
- `platformio.ini` — PlatformIO project configuration

## Getting Started
1. Clone this repository or copy the project to your PlatformIO workspace.
2. Open with VS Code and PlatformIO extension.
3. Connect your slave hardware (e.g., ESP32-C3 or compatible MCU).
4. Configure any hardware-specific settings in `platformio.ini` or source files as needed.
5. Build and upload the firmware using PlatformIO.

## Configuration & Usage
- Connect the slave to the master using a flat 6-core telephone cable (terminated with 4-pin JST XH2.54 connectors).
- To configure the slave address or update firmware:
  - Press the button to activate the web portal.
  - Connect to the "SleepyLoRaSlave" WiFi AP (password: blind1234).
  - Access the configuration page to set the slave number or upload new firmware.

## Communication Protocol
- **RS485 UART**: Used to communicate with the master device. Each slave has a unique address.
- **Command Structure**: See `include/Command_Register.h` for command codes and protocol details.

## License
SPDX-License-Identifier: MIT

Copyright (c) 2025 Chris Huitema

---
For more information, see the source code and comments in `src/main.cpp` and related files.
