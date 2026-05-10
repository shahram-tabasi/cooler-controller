# 🧊 CoolerController – Version 2

> IN THE NAME OF ALLAH

---

## 📌 Project Overview

**CoolerController_Version2** is an embedded cooling system controller  
designed using **two ATmega64A microcontrollers** operating in a Master–Slave architecture.

The system provides intelligent timing, temperature control, remote operation,
and motor frequency control features.

---

## 🏗️ System Architecture

The project is built around **two ATmega64A AVR microcontrollers**:

### 🔹 Master MCU
Responsible for:
- Main control logic
- Push button inputs
- Time scheduling management
- Temperature processing (LM35)
- Remote control decoding
- Motor frequency control
- Sending display data to Slave MCU

### 🔹 Slave MCU
Responsible for:
- Driving 7‑Segment displays
- Receiving display data from Master MCU
- Handling multiplexing and display refresh

The Slave acts purely as a display driver, while all decision-making logic
is handled by the Master.

---

## 🛠️ System Specifications

| Parameter | Value |
|-----------|--------|
| **Microcontroller** | 2 × ATmega64A |
| **Architecture** | Master–Slave |
| **Core Frequency** | 16.000000 MHz |
| **Memory Model** | Small |
| **External RAM** | 0 |
| **Data Stack Size** | 1024 |

---

## ✨ Features

- ⏱️ Programmable ON/OFF timing
  - Periodic mode
  - Weekly scheduling
- 🌡️ Automatic temperature control using **LM35 temperature sensor**
- 🎮 Remote control operation
- 🔘 Manual control via push buttons
- ⚡ Motor frequency control (speed control)
- 🔢 7-Segment display interface (driven by Slave MCU)

---

## 🌡️ Temperature Control

The system uses an **LM35 analog temperature sensor** to:

- Measure ambient temperature
- Automatically adjust cooling operation
- Maintain configured temperature thresholds

---

## ⏰ Scheduling Capabilities

- Periodic ON/OFF control
- Weekly programmable schedule
- Automatic activation/deactivation based on user-defined timing

---

## 🚀 Getting Started

### 🔧 Requirements

- AVR-GCC / CodeVisionAVR
- AVR Programmer (e.g., USBasp)
- 2 × ATmega64A Microcontrollers
- 16 MHz Crystal Oscillators
- LM35 Temperature Sensor
- 7-Segment Display Module
- Remote Control Receiver Module

### 🔨 Build & Flash

1. Compile Master and Slave firmware separately.
2. Generate HEX files.
3. Flash each HEX file to the corresponding MCU.
4. Configure fuse bits for external 16 MHz crystal.
5. Power up the system.

---

## 📂 Suggested Repository Structure

CoolerController_Version2/

│

├── Master/

│ └── source files

├── Slave/

│ └── source files

├── docs/

├── hardware/

└── README.md


---

## 📝 Notes

- Ensure proper communication protocol between Master and Slave (UART/SPI/I2C as implemented).
- Verify stable 16 MHz clock configuration.
- Stack size configured to 1024 bytes.
- No external RAM used.

---

## 📜 License
Shokuhi Industrial co.
