# SleepyLora Blinds System

A low-power wireless smart blinds system using LoRa to enable low power yet fast response to commands - 100% local control.

In July 2023 I set out the key requirements for the project
- 100% local control - no 3rd party servers etc.
- solar powered with up to 3days bad weather.
- back drive able without breaking.
- secure
- be cheap

This system is 100% local, no external access required at all
A 2.5w 380mm x 70mm thin film flexible panel is enough to keep a 2000mah battery fully charged with less than 2 hours a day of sun. the 2000mah battery lasts about 10 days with no panel connected.
Key design requirement was for the blinds to still operate by hand, so a mechanism was developed that disengages the motor from the actuator once the move command is complete. 
Position is sensed using a linear potentiometer thats connected to the actualing arm, so the blinds always know their position.
All data packets are AES128 encrypted and hardened against replay attacks using 1 second interval TOTP, so no-one can open your blinds but you.
Costs are kept low by using cheap modules form china, assembled onto custom PCB's and housed in 3d printed parts.
The wired slaves are powered by the master, thus only a single battery and solar panel per system and a significant reduction in actuator cost too. 

This project includes electronics designs, firmware, and 3D printable parts for a complete DIY solution. 

This has been a couple of years in the making, and still very much a work in progress. If you find this project useful, please consider supporting my work:

[![Buy Me a Coffee](https://img.shields.io/badge/Buy%20me%20a%20coffee-orange?logo=buy-me-a-coffee&logoColor=white)](https://www.buymeacoffee.com/chrishuitema)

Your support helps me keep building and maintaining open-source projects like this. Thank you!

---

### System

- **Gateway**:  
  Heltec Wireless Stick Lite V3, Connects to WiFi and MQTT, relays commands between Home Assistant and LoRa network. 
  Exposes a web portal for configuration and security key generation

- **Master**:  
  Blind actuator with a Heltec Wireless Stick Lite V3, voltage booster, motor driver, position sensor and RS485 module.
  Powered from a single cell lipo battery or USB.
  Can supply power to and control multiple slave devices via RS485.
  Exposes a web portal for configuration and firmware updates. (via button press or mqtt command)

- **Slave**:  
  Blind actuator with an ESP32-c3 module, motor driver, position sensor and RS485 module. 
  Powered by the Master
  Exposes a web portal for address configuration and OTA updates. (via button press or mqtt command)
  Connects to the master using a flat 6core telephone cable terminated with 4pin JST XH2.54 connectors
  
---

## Features

- **Long-range LoRa wireless communication**
- **Easy WiFi and MQTT Setup**
- **Easy key generation and sharing**
- **Home Assistant auto-discovery**
- **OTA firmware updates**
- **Configurable RS485 slave addressing**
- **Low-power operation** 
- **3D-printable hardware** 
- **Easy to assemble electronics**
- **Open-source** 

---

## Project Structure

- **3d_prints/**  
  STL files for all mechanical parts 

- **Electronics/**  
  KiCad schematics, PCB layouts, and Gerber files for the master and slave boards.

- **SleepyLoraGateway/**  
  Firmware for the LoRa gateway device. Handles MQTT, Home Assistant integration, and LoRa communication. 

- **SleepyLoraMaster/**  
  Firmware for the master device, an actuator for one set of shutters, with the capability to power and control several slave devices via RS485.

- **SleepyLoraSlave/**  
  Firmware for the slave device, an actuator for one set of shutters and reports position/status.

---

## Getting Started

- Order all the parts/modules from Aliexpress, list in `Electronics/`
- Order PCB/s using the KiCad files in `Electronics/`
- Print and assemble the required parts from `3d_prints/`
- Solder the modules onto the PCB's
- Install electronics in 3d printed base
- Flash the appropriate firmware to each device:
  - Gateway: `SleepyLoraGateway/`
  - Master: `SleepyLoraMaster/`
  - Slave: `SleepyLoraSlave/`
- Configure the Gateway
  - Connect to the SleepyLoraGateway AP (pw: blind1234)
  - Set the WiFi, MQTT and NTP server information.
  - Set LoRa Region
  - Generate keys and download them. 
  - Save and reboot.
  - Gateway Connects to WiFi, MQTT and Sync's its time.
  - Gateway Publishes gateway metric discovery info and publishes data.
- Configure the Master/s
  - Connect to the SleeplyLoRaMaster AP (pw: blind1234)
  - Load the keys file downloaded above
  - Save and reboot
  - Master sync's time with gateway
  - Master publishes Device matric discovery info - battery, diagnostic data etc every 60sec.
  - Master publishes Blind discovery info, and publishes data every 60sec.
- Check your HA MQTT integration, the Gatway, device and blind will magically appear after a minute or so. 
- Optionally - configure the slave/save
  - Connect the slave to the master 
  - Press the reset button on the master
  - Master will scan for slaves on initial bootup
  - Master publishes the newly found slave.
  - Optionally - if a second slave is required
    - press the "Open Web Portal" button on the slave device in HomeAssistant
	- Connect to the SleeplyLoRaSlave AP (pw: blind1234)
	- Change the slave number
	- Save to flash
    - Press the reset button on the master
    - Master will scan for slaves

---

## Documentation

- Each firmware folder contains its own README with build and usage instructions.
- For wiring, assembly, and setup, see my videos on youtube or open an issue for help.

---

## License

This project is open-source under the MIT License.  
See [LICENSE](LICENSE) for details.

---

## Credits

- Hardware, firmware, and 3D design by Chris Huitema
- Built with [PlatformIO](https://platformio.org/), [KiCad](https://kicad.org/), and [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)

---

**Happy hacking and smart shading!**