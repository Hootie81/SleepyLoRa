# Electronics Folder — SleepyLoRa Project

This folder contains all the electronics design files, schematics, PCB layouts, and bills of materials (BOM) for the SleepyLoRa Blinds System. The electronics are designed to be easy to assemble, cost-effective, and reliable for DIY smart blinds control.

## Structure
- **bom.csv** — Common components and extras used across all devices (master, slave, gateway, etc.).
- **Master/** — Schematics, PCB layout, and device-specific BOM for the SleepyLoRaMaster (blind actuator/master controller).
- **Slave/** — Schematics, PCB layout, and device-specific BOM for the SleepyLoRaSlave (slave blind actuator).


Each device folder contains:
- KiCad project files (`.pro`, `.sch`, `.kicad_pcb`, etc.)
- Gerber files for PCB manufacturing
- Device-specific BOM (bill of materials)

## How to Use
1. Review `bom.csv` for the list of common components and extras needed for all devices.
2. For each device (Master, Slave, Gateway):
   - Open the corresponding folder for full schematics, PCB layout, and device-specific BOM.
   - Use the KiCad files to view or modify the design.
   - Use the Gerber files to order PCBs from your preferred manufacturer.
   - Use the device-specific BOM to order parts for that device.
3. Assemble the PCBs according to the schematics and BOMs.

## Notes
- All designs are open-source and provided as-is for DIY use.
- If you make improvements or modifications, please consider contributing back to the project!

## License
SPDX-License-Identifier: MIT

---
For questions or support, open an issue on the main [SleepyLoRa GitHub repository](https://github.com/Hootie81/SleepyLoRa).
