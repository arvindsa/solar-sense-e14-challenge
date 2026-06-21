# SolarSense: Smart Solar Panel Monitoring System

A comprehensive IoT system for real-time solar panel soiling detection using multi-sensor fusion and edge AI. This project monitors three solar panels -one kept clean as a reference baseline to quantify soiling-induced efficiency losses in real time.

## Project Overview

SolarSense uses a clean reference panel to cancel out weather effects completely (both panels see the same sky), leaving only soiling-induced losses measurable. The system combines:

- **Hardware**: Custom STM32L476 sensor board with CAN telemetry
- **Gateway**: Arduino UNO Q with dual STM32U585 (bare-metal HAL) and Linux bridge
- **Dashboard**: LabVIEW-based live visualization
- **AI**: Edge Impulse model for clean/alert/soiled classification

**Current Status**: Phase 1 (data collection and live dashboard) complete. Full pipeline implemented but pending end-to-end hardware validation.

## Repository Structure

```
firmware/
├── solar-sense-stm32l476/       # Main sensor board (STM32L476RGT6)
├── arduinoq-stm32u585/          # Gateway STM controller (bare-metal HAL)
│   └── bridge/                  # CAN frame bridging logic
└── arduinoq-linux/              # Gateway Linux side (Python bridge)

labview/                         # Dashboard VIs for Window
hardware/                        # PCB schematics and production files
```
