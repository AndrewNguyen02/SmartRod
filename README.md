# SmartRod (Cyber Rod)

A rod-mounted fishing telemetry system that logs casts and distance, detects bite-like motion, and stores sessions for later review. Built around an ESP32-class MCU with a compact on-rod UI and optional Bluetooth sync to a mobile companion app for visualization and metrics.


## Overview

- Cast detection + distance estimation using a **color sensor** reading a **marked reel/spool**. *(Sensor method may change to a hall sensor + magnet for a more compact build.)*
- **IMU-based bite/motion alerts** for strike-like rod movement events.
- On-rod **LCD** for time, basic weather/context, battery %, and session status.
- Local session logging with optional Bluetooth sync to a phone app.
- Mobile app goals: **log session data**, **visualize distance**, and display **temperature + other contextual metrics**.
- Designed for student-friendly fabrication (3D printing/basic CNC) and serviceable sealing (gaskets/O-rings).


## Features

- **Cast & Distance Logging:** Detects casts and estimates line payout by counting spool events (currently via color-mark passes; may transition to hall sensor + magnet).
- **Bite/Motion Alerts:** IMU monitors rod motion patterns and triggers alerts on strike-like events.
- **User Interface:** Simple, glove-friendly controls with a readable on-rod LCD.
- **Session Storage:** Stores multiple sessions for post-session review.
- **Connectivity (optional):** BLE sync to a smartphone app for session logging and visualization.
- **Mobile App (planned):**
  - **Log data in the phone app** (session history, events, summaries)
  - **Distance visualization** (per-cast and/or per-session views)
  - **Temperature and other metrics** (contextual conditions tied to sessions)
- **Export:** CSV/GPX output for analysis and sharing (via app/export flow).


## Hardware & Tools

- **MCU:** ESP32-class (ESP32-S3 supported; final target confirmed during prototyping)
- **Sensors:** Color sensor (spool mark) / *(possible hall sensor + magnet)*, IMU (bite/motion)
- **UI:** Compact LCD + buttons
- **Power:** 3.7 V Li-ion + USB-C charging + regulation
- **CAD:** Tinkercad
- **Firmware:** Arduino IDE + VS Code (PlatformIO optional)
- **Version Control/CI:** GitHub


## Project Structure

- `firmware/` — MCU firmware (sensing, UI, logging, BLE)
- `hardware/` — wiring diagrams, pin maps, enclosure notes
- `cad/` — enclosure/brackets (STLs and source files)
- `app/` — mobile companion app (sync + visualization + export)
- `docs/` — requirements, acceptance tests, field test plans


## Architecture

- **Sensing pipeline:** Spool sensor (color mark or hall+magnet) → cast events + distance estimate  
- **Alert pipeline:** IMU → bite/motion classification → user alert  
- **UI pipeline:** Session status → LCD + buttons  
- **Data pipeline:** Session logs → local storage → (optional) BLE sync → phone app (logging + visualization + metrics) → export (CSV/GPX)


## Build & Run

- Build steps and setup guides will live in `docs/` as the MVP solidifies.
- MVP demo goals:
  - Cast logging rate + distance repeatability
  - IMU alert tuning (latency + false positives)
  - Session storage and retrieval
  - Optional BLE sync
  - App-side session logging + basic distance visualization


## Testing

- **Bench:** spool event counting consistency, IMU event detection tuning, UI readability, battery runtime.
- **Field:** cast logging percentage, distance accuracy targets, session retrieval, optional BLE sync success rate.


## Notable Design Choices

- **Distance via spool sensor:** simple, low-cost method for estimating line payout without complex mechanics (color sensor initially; hall sensor is a compact alternative).
- **IMU for bite alerts:** detects strike-like rod motion without putting electronics in-line with the fishing line.
- **Serviceable waterproofing:** gaskets/O-rings prioritized over heavy potting for rework and maintenance.
- **MVP-first scope:** core logging + alerts + UI + storage before higher-level mapping features.


## Roadmap

- **MVP:** cast detection + distance, IMU alerts, LCD status, local storage, optional BLE sync, basic app logging + distance visualization
- **Stretch:** bobber position estimation (distance + GPS/direction), richer analytics, refined chassis, deeper exports, expanded app metrics (temperature + context)


## Credits

CECS 490 Senior Project — SmartRod / Cyber Rod team.
