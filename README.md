# Waveshare RP2040 Sensor Relay Controller

A MicroZig-based project for Waveshare MCU boards with RP2040 chip that controls a relay based on sensor input and provides visual feedback through WS2812 LEDs.

## Features

- **Sensor Monitoring**: Reads analog sensor data via ADC (GPIO28/AIN2)
- **Relay Control**: Controls a relay on GPIO29 based on sensor threshold
- **LED Feedback**: 5x5 WS2812 LED matrix on GPIO16 for visual status indication
- **Automatic Operation**: Continuous monitoring with 1-second intervals

## Hardware Requirements

- Waveshare MCU board with RP2040 chip
- WS2812 LED strip/matrix (25 LEDs in 5x5 configuration)
- Relay module
- Analog sensor (connected to GPIO28)
- Appropriate power supply for LEDs and relay

## Pin Configuration

| GPIO   | Function      | Direction   |
| ------ | ------------- | ----------- |
| GPIO16 | WS2812 LEDs   | Output      |
| GPIO28 | Sensor Input  | Input (ADC) |
| GPIO29 | Relay Control | Output      |

## Operation

1. **Initialization**: Sets up GPIO pins, ADC, and PIO for WS2812 control
2. **LED Display**: Shows green LEDs initially, then blue during operation
3. **Sensor Reading**: Continuously reads analog sensor value
4. **Relay Logic**:
   - If sensor value < 2000: Relay OPEN
   - If sensor value ≥ 2000: Relay CLOSED
5. **Status Indication**: LEDs change to red when program ends

## Building

This project uses MicroZig for RP2040 development.

```bash
zig build
```
