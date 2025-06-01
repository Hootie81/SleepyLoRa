# SleepyLoraGateway

SleepyLoraGateway is the gateway firmware for the SleepyLora Blinds System. It acts as a bridge between the local LoRa network of blind actuators and your home automation system (such as Home Assistant) via WiFi and MQTT. The gateway provides secure, local, and reliable communication for smart blind control.

## Features
- Connects to WiFi and MQTT broker for integration with Home Assistant
- Relays commands and status between Home Assistant and LoRa network
- Exposes a web portal for configuration and security key generation
- Handles LoRa communication with master devices
- AES128 encrypted packets with replay protection
- 100% local control, no third-party servers required

## Project Structure
- `src/` — Main source code for the gateway
- `include/` — Shared headers and protocol definitions (e.g., Command_Register.h)
- `lib/` — External and custom libraries
- `platformio.ini` — PlatformIO project configuration

## Getting Started
1. Clone this repository or copy the project to your PlatformIO workspace.
2. Open with VS Code and PlatformIO extension.
3. Connect your gateway hardware (e.g., Heltec Wireless Stick Lite V3).
4. Configure WiFi, MQTT, and LoRa region in the web portal after flashing.
5. Build and upload the firmware using PlatformIO.

## Configuration & Usage
- After flashing, connect to the "SleepyLoraGateway" WiFi AP (password: blind1234).
- Access the web portal to set up WiFi, MQTT, NTP, and LoRa region.
- Generate and download security keys for the system.
- Save and reboot the gateway to connect it to your home network and MQTT broker.
- The gateway will automatically publish device and blind discovery info for Home Assistant.

## Communication Protocol
- **LoRa**: Communicates with master devices using encrypted packets.
- **WiFi/MQTT**: Integrates with Home Assistant and other automation systems.
- **Command Structure**: See `include/Command_Register.h` for command codes and protocol details.

## License
SPDX-License-Identifier: MIT

Copyright (c) 2025 Chris Huitema

---
For more information, see the source code and comments in `src/main.cpp` and related files.
