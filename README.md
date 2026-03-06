# LoRa Node

An open-source LoRa(Long Range) communication node designed for wireless communication experiments and embedded system research.

This repository contains **complete hardware design, embedded firmware, and host-side software** for a standalone LoRa node platform.

The system is based on **STM32F103 + SX1268 (E22-400M22S)** and supports flexible LoRa configuration for experimentation.

---

## Features

- STM32F103C8T6 microcontroller
- SX1268 LoRa radio module (E22-400M22S)
- 433 MHz LoRa communication
- Full support for configurable LoRa parameters:
  - Spreading Factor (SF)
  - Bandwidth (BW)
  - Coding Rate (CR)
- 0.91" OLED display
- 4×4 keypad for local interaction
- UART interface for host communication
- Fully open hardware design
- Host-side software

This platform can be used for:

- LoRa communication experiments
- wireless protocol research
- RF testing
- embedded systems development
- LPWAN experimentation

---

## Hardware Architecture

Main components:

- **MCU:** STM32F103C8T6
- **LoRa module:** SX1268 (E22-400M22S)
- **Display:** 0.91" OLED
- **Input:** 4×4 tact-switch keypad
- **Host interface:** UART
- **Frequency band:** 433 MHz

System structure:


STM32F103
│
├── SPI → SX1268 LoRa module
│ └── Antenna
│
├── OLED display
│
├── 4×4 keypad
│
└── UART → Host computer

---

## Hardware Design

The PCB is designed using **EasyEDA** (JLC EDA Pro).

Hardware files are located in:


hardware/


The project includes:

- schematic
- PCB layout
- BOM
- signal integrity considerations
- power integrity considerations

---

## Firmware

The firmware runs on **STM32F103** and is developed using:

**STM32CubeIDE for VSCode**

### Requirements

- STM32CubeIDE for VSCode
- ST-Link programmer/debugger

### Build and Flash

1. Install STM32CubeIDE for VSCode following ST's official instructions.
2. Connect the board via **ST-Link**.
3. Open the `firmware` folder in VSCode.
4. Use the VSCode build and debug tasks to compile and program the device.

---

## Host Software

The `host` directory contains a **PC-side program** used to communicate with the LoRa node via UART.

It is written in **C++** and uses **CMake**.

### Build with MinGW


cd host
build_mingw.bat


### Build with MSVC

Run in "Developer Command Prompt for VS":
cd host
build_msvc.bat


After building, run the generated executable to interact with the node.

---

## LoRa Configuration

The firmware supports flexible configuration of LoRa parameters:

- **Frequency:** 433 MHz
- **Spreading Factor:** configurable
- **Bandwidth:** configurable
- **Coding Rate:** configurable

This allows experimentation with different LoRa communication setups.

---

## Project Status

Current status:

- Hardware design completed
- Firmware implemented
- Host communication program implemented

The platform is suitable for further development and experimentation.

---

## License

This project is released under the terms of the license included in this repository.

---

## Keywords

LoRa  
Long Range  
SX1268  
STM32  
Embedded Systems  
IoT  
RF  
LPWAN  
Wireless Communication
