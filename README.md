# NMEA Reader 2.0" TFT

NMEA 0183 Sensor Monitor on ESP32 + 2.0" ST7789 TFT.

Developed by **Loxodrome Tech**.

---

## Overview

NMEA Reader is a standalone hardware monitor that listens on a boat's NMEA 0183
bus and shows, at a glance, which sensor categories are currently talking —
without needing a laptop or a Wi-Fi/web UI in the loop.

---

## What it does

- Reads NMEA 0183 sentences over UART2 (GPIO16/17 by default)
- Automatic baud rate detection — cycles through the common NMEA 0183 rates
  at boot (and re-scans if the bus goes silent) and locks on by validating
  sentence checksums, no manual configuration needed
- Classifies traffic into sensor categories (GPS, AIS, Heading, Gyro,
  Velocity, Radar, Sounder, Weather, Xducer) from the sentence formatter,
  falling back to the talker ID for anything not recognized
- Lists only the categories currently detected — a sensor drops off the list
  a few seconds after it stops transmitting
- Landscape 320x240 display, laid out to fill the screen with however many
  sensors are present, with a debounced/stable layout (no flicker/jitter as
  sensors are discovered)

---

## Hardware

- ESP32 WROOM32
- 2.0" 7-pin SPI TFT, ST7789 driver, 240x320 panel (run in landscape)
- NMEA source wired to UART2: RX on GPIO16, TX on GPIO17 (only needed if the
  source expects a request)

Pins and SPI speed are configurable via `build_flags` in `platformio.ini`.

---

## Use Cases

- Quick onboard check of which instruments are actually on the bus
- Bench testing / commissioning navigation equipment (GPS, AIS, wind, depth)
- Field diagnostics without a laptop

---

## Repository Structure

```
src/            Firmware source (PlatformIO / Arduino framework)
platformio.ini  Build configuration, pins, board
assets/         Splash logo preview
```
