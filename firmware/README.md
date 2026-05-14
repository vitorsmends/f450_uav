# F450 UAV Firmware

## Overview

This repository contains the embedded firmware responsible for low-level motor actuation and onboard IMU acquisition for the F450 UAV platform.

The firmware runs on an ESP32 and provides:

- Brushless motor control through ESCs
- Serial communication with a Raspberry Pi
- Real-time IMU acquisition from the GY-91 module
- Communication timeout failsafe
- Modular architecture for future ROS 2 integration

The Raspberry Pi acts as the high-level computer, while the ESP32 is responsible for deterministic low-level control.

---

# System Architecture

```text
Raspberry Pi
    |
    | Serial UART / USB
    v
ESP32 Firmware
    |
    +-- ESC Manager
    |     +-- ESC 1
    |     +-- ESC 2
    |     +-- ESC 3
    |     +-- ESC 4
    |
    +-- GY-91 IMU
```

---

# Features

- 4-motor ESC control
- RPM-based command interface
- Modular ESC abstraction
- Serial motor command protocol
- IMU telemetry streaming
- Communication watchdog
- ROS 2 ready architecture
- Arduino IDE compatible

---

# Repository Structure

```text
firmware/
├── firmware.ino
├── RPMESCController.hpp
├── RPMESCController.cpp
├── ESCManager.hpp
├── ESCManager.cpp
├── SerialMotorProtocol.hpp
├── SerialMotorProtocol.cpp
├── GY91IMU.hpp
└── GY91IMU.cpp
```

---

# Hardware

## Main Components

| Component | Description |
|---|---|
| ESP32 | Main embedded controller |
| ESCs | Brushless motor controllers |
| Brushless Motors | UAV propulsion |
| GY-91 | IMU module (MPU9250 + BMP280) |
| Raspberry Pi | High-level onboard computer |
| 3S Battery | Main power source |

---

# Pin Mapping

## ESC Connections

| ESC | ESP32 GPIO |
|---|---|
| ESC 1 | GPIO18 |
| ESC 2 | GPIO19 |
| ESC 3 | GPIO25 |
| ESC 4 | GPIO26 |

---

## IMU Connections

| GY-91 Pin | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

---

# Motor Command Protocol

The firmware receives motor commands from the Raspberry Pi through Serial communication.

## Packet Format

### Motor Command

```text
<MOTOR,rpm1,rpm2,rpm3,rpm4>
```

Example:

```text
<MOTOR,1200,1200,1200,1200>
```

---

### Stop Command

```text
<STOP>
```

---

# IMU Telemetry

The ESP32 periodically transmits IMU measurements back to the Raspberry Pi.

## Telemetry Format

```text
<IMU,time_ms,ax,ay,az,gx,gy,gz>
```

Example:

```text
<IMU,12450,0.02,-0.01,1.00,0.10,-0.05,0.02>
```

---

# Safety Features

## Communication Watchdog

If no valid motor command is received within the configured timeout period:

```cpp
#define COMMAND_TIMEOUT_MS 500
```

all motors are automatically stopped.

This prevents uncontrolled behavior in case of:
- Raspberry Pi crash
- Serial disconnection
- ROS 2 node failure
- Communication interruption

---

# ROS 2 Integration

The firmware is designed for future ROS 2 integration.

Expected architecture:

```text
ROS 2 Node
    |
    +-- Publishes motor RPM setpoints
    |
    +-- Reads IMU telemetry
```

The Raspberry Pi should continuously stream motor commands to the ESP32 at approximately:

```text
20 Hz to 50 Hz
```

---

# Build Environment

Currently supported:

- Arduino IDE
- ESP32 Arduino Core

Future migration target:

- PlatformIO
- micro-ROS
