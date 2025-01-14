# Open Incubator Temperature Control System

An ESP32-based temperature control system for a PHOENIX NOC-100 open incubator, featuring IoT capabilities and a touchscreen interface.

## Overview

<div align="center">
  <img src="images/completed_device.JPEG" alt="Completed Device" width="600">
</div>

*Figure: Completed device based in Phoenix NOC100.*

This project implements a digital temperature control system for a neonatal open incubator. It includes:

- PID temperature control with 0.1°C resolution
- 7-inch touchscreen HMI interface
- IoT monitoring and control capabilities
- Real-time temperature monitoring and alarms
- Emergency stop functionality

## Hardware Components

- ESP32 microcontrollers (2x)
- Elecrow CrowPanel 7.0" HMI Display
- Draeger MU06951 skin temperature sensor
- TRIAC-based power control circuit
- Custom PCB for signal conditioning and power management

## Features

### Temperature Control
- Operating range: 36.0°C - 38.0°C
- Resolution: 0.1°C
- Accuracy: ±0.3°C (compliant with NTC-IEC 60601-2-21)
- Digital PID control with anti-windup

### Safety Features
- Emergency stop button
- Multiple alarm conditions:
  - High temperature (>39°C)
  - Low temperature (<35°C)
  - Sensor failure
  - Heater failure
  - Emergency stop activation

### IoT Capabilities
- Remote temperature monitoring
- Setpoint adjustment
- Real-time data logging
- Web interface for medical staff
- Engineering access mode

## Software Architecture

The system consists of two main components:

### Control Module (ClosedLoop.ino)
- Temperature measurement and filtering
- PID control implementation
- TRIAC firing control
- ESP-NOW communication
- Blynk IoT integration

### HMI Module (HMI.ino)
- Touch interface management
- Alarm display
- Temperature and power visualization
- ESP-NOW communication with control module

## Installation

1. Clone this repository
2. Install required libraries:
   - ESP32 Arduino Core
   - LVGL
   - Arduino_GFX_Library
   - Blynk
   - ESP-NOW
3. Configure WiFi credentials and Blynk tokens in both .ino files
4. Upload ClosedLoop.ino to the control ESP32
5. Upload HMI.ino to the display ESP32

## Dependencies

- **ClosedLoop.ino**:
  - Blynk IoT platform: [Blynk Documentation](https://docs.blynk.io)
  - ESP-NOW: [ESP-NOW Documentation](https://www.espressif.com)

- **HMI.ino**:
  - LVGL: [LVGL Documentation](https://lvgl.io)
  - Arduino GFX Library: [Arduino_GFX Documentation](https://github.com/moononournation/Arduino_GFX)

## Usage

1. Power on the system
2. Wait for the initialization sequence
3. Set desired temperature using the touchscreen
4. Monitor temperature and alarms through the display
5. Use web interface for remote monitoring (optional)

## Contributing

This project is part of a thesis work for the Universidad Autónoma de Bucaramanga. Contributions are welcome through pull requests.

## Documentation & Resources

- [Complete Project Documentation](http://hdl.handle.net/20.500.12749/26793) - Detailed technical documentation and implementation details
- [PCB Design Files](https://oshwlab.com/nico323vera/proyectodegradoincubadorafinalcopy_2024-04-15_17-19-56) - Circuit schematics and PCB layout files

## Authors

- Nicolás Vera Amador
- Carlos Daniel Yi Villamizar

## Acknowledgments

- M.Sc. Hernando Gonzalez Acevedo (Director)
- Clínica FOSCAL Internacional

## 
