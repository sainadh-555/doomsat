# GARUDA-1U: 1U CubeSat Digital Twin & Mission Simulation

[![Python 3.11](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/downloads/release/python-3119/)
[![Architecture](https://img.shields.io/badge/architecture-1U%20CubeSat-orange.svg)]()
[![Status](https://img.shields.io/badge/development-All%20Phases%20Complete%20(108%2F108%20Tests)-brightgreen.svg)]()
[![Tests](https://img.shields.io/badge/tests-108%20passed-success.svg)]()
[![Live Dashboard](https://img.shields.io/badge/Live%20Dashboard-doomsat.onrender.com-00d4ff.svg)](https://doomsat.onrender.com/)

GARUDA-1U is an engineering-grade digital twin and multi-physics mission simulator for a 1U CubeSat ($100 \times 100 \times 100\text{ mm}$, mass $\le 1.330\text{ kg}$), developed with high-fidelity aerospace and embedded systems modeling in Python.

**Live Mission Control Dashboard:** **[https://doomsat.onrender.com/](https://doomsat.onrender.com/)**

---

## 1. System Architecture & Hardware Baseline

```
+-----------------------------------------------------------------------------------+
|                                  DOOMSat 1U CubeSat                               |
|                                                                                   |
|  +---------------------+      +---------------------+      +-------------------+  |
|  |     OBC Subsystem   | UART |   TT&C Controller   | SPI  |   RF Transceiver  |  |
|  | Raspberry Pi Zero 2W|<---->|    Espressif ESP32  |<---->|   Semtech SX1276  |  |
|  +----------+----------+      +----------+----------+      +---------+---------+  |
|             | MIPI CSI-2                 | I2C                       | LoRa RF    |
|             v                            v                           v (Antenna)  |
|  +---------------------+      +---------------------+                |            |
|  |     Camera Payload  |      |   ADCS & Env Sensors|                |            |
|  |   OV5647 5MP Sensor |      | - MPU6050 (IMU)     |                |            |
|  +---------------------+      | - LIS3MDL (Mag)     |                |            |
|                               | - 6x BPW83 Photodiode                |            |
|  +---------------------+      |   via 2x ADS1115 ADC|                |            |
|  |     EPS & Thermal   |      | - BME280 (Health)   |                |            |
|  | - 18650 Li-ion Cell |      +---------------------+                |            |
|  | - Body-mount GaAs PV|                                             |            |
|  +---------------------+                                             v            |
+----------------------------------------------------------------------+------------+
                                                                       | RF Link
                                                                       v
                                                       +----------------------------+
                                                       |   Ground Station System    |
                                                       |  - ESP32 + SX1276 Receiver |
                                                       |  - Mission Control Backend |
                                                       |  - Image Reconstructor    |
                                                       +----------------------------+
```

### Reference Hardware Specifications
- **Structure:** 1U Standard ($100 \times 100 \times 100\text{ mm}$, mass budget $\le 1.33\text{ kg}$).
- **On-Board Computer (OBC):** Raspberry Pi Zero 2 W (Quad-Core 64-bit ARM Cortex-A53 @ 1.0 GHz, 512 MB LPDDR2).
- **Payload:** OmniVision OV5647 5MP CMOS camera with MIPI CSI-2 interface.
- **TT&C Controller:** Espressif ESP32-WROOM-32D (Dual-core Xtensa @ 240 MHz, 520 KB SRAM).
- **RF Transceiver:** Semtech SX1276 LoRa transceiver module (Amateur / ISM bands, $+20\text{ dBm}$ P.A.).
- **Attitude Determination & Control (ADCS):**
  - IMU: TDK InvenSense MPU6050 (3-axis gyroscope + 3-axis accelerometer).
  - Magnetometer: STMicroelectronics LIS3MDL (3-axis digital magnetic sensor).
  - Sun Sensors: 6 $\times$ Vishay BPW83 silicon PIN photodiodes (one per CubeSat face) read by 2 $\times$ Texas Instruments ADS1115 16-bit ADCs.
- **Environmental Sensor:** Bosch Sensortec BME280 (Temperature, barometric pressure, relative humidity).
- **Electrical Power Subsystem (EPS):** 3.7V 2600 mAh 18650 Li-ion cell, body-mounted triple-junction solar panels, regulated 3.3V and 5.0V power buses.

---

## 2. Engineering Principles

- **Rigorous SI Units:** All calculations strictly use SI units (meters, kilograms, seconds, Kelvin, Joules, Watts, radians).
- **Documented Physics:** Every simulation domain is derived from first-principles mechanics, orbital dynamics, thermodynamics, and electromagnetics.
- **Traceable Datasheets:** Hardware parameters originate from verified manufacturer datasheets (categorized as *Datasheet-Derived*, *Assumed*, or *Measured*).
- **Regulatory Compliance:** Adherence to Indian Wireless Planning & Coordination (WPC) delicensed rules (865–867 MHz) and ITU / IARU Amateur Satellite Service rules (435–438 MHz).
- **Zero Fabrication:** Test results and sensor measurements are never fabricated.

---

## 3. Development Roadmap

- [x] **Phase 1: Architecture, Requirements, Repository Foundation & Astrodynamics**
- [x] **Phase 2: Mechanical Assembly & Structural Mass/Inertia Properties**
- [x] **Phase 3: Attitude Determination and Control Subsystem (ADCS)**
- [x] **Phase 4: Payload Subsystem (Optical Camera & Image Pipeline)**
- [x] **Phase 5: Telemetry, Tracking & Command (TT&C) Comms & Protocol**
- [x] **Phase 6: Electrical Power Subsystem (EPS) & Thermal Control (TCS)**
- [x] **Phase 7: Ground Station Segment & Mission Control Dashboard**
- [x] **Phase 8: Complete System Integration, Full-Orbit Simulation & Hardware-in-the-Loop**

---

## 4. Quickstart & Unified CLI

GARUDA-1U includes a unified CLI for mission simulation, hardware-in-the-loop benchtop testing, and launching the Mission Control Dashboard:

### 1. Run Full-Orbit Multi-Physics Simulation
Simulate a complete 95-minute orbit (5700s) with 6-DoF attitude dynamics, B-dot detumbling, solar/eclipse ephemeris, EPS battery balance, and 8-node thermal network:
```bash
python -m doomsat.sim.cli sim --duration 5700 --dt 5.0
```

### 2. Launch Mission Control Dashboard
You can view the live cloud deployment directly at **[https://doomsat.onrender.com/](https://doomsat.onrender.com/)**, or run locally:
```bash
python -m doomsat.sim.cli dashboard --host 127.0.0.1 --port 8000
```
Open **[http://127.0.0.1:8000/](http://127.0.0.1:8000/)** in your browser.

### 3. Run Hardware-in-the-Loop (HIL) Benchtop Test
Connect a physical ESP32 board running CSTP firmware over UART (enforcing Indian WPC G.S.R. 564(E) delicensed frequency at 865.500 MHz):
```bash
python -m doomsat.sim.cli hil --port COM3 --baud 115200
```

---

## 5. Repository Structure

```text
doomsat/
├── docs/                      # Systems Engineering documentation
│   ├── MRD.md                 # Mission Requirements Document
│   ├── SAD.md                 # System Architecture Document
│   ├── ICD.md                 # Interface Control Document
│   ├── COMPONENT_REGISTER.md  # Traceable hardware component register
│   └── RF_REGULATORY.md       # RF spectrum & regulatory plan
├── src/
│   └── doomsat/
│       ├── core/              # SI constants, coordinate frames, schemas
│       ├── config/            # YAML configs & Pydantic validation loader
│       ├── sim/               # Master orchestrator, CLI, HIL test runner
│       ├── mechanical/        # Mass budget & inertia tensor (STEP/STL export)
│       ├── obc/               # RPi flight software FSM, commands & watchdog
│       ├── payload/           # OV5640 camera & image fragmentation pipeline
│       ├── comms/             # CSTP protocol, SLIP framing & SX1276 LoRa PHY
│       ├── orbit/             # Cowell RK4 with J2, SGP4 TLE, ephemeris
│       ├── adcs/              # 6-DoF attitude dynamics, B-dot control, sensors
│       ├── eps/               # Solar generation, 18650 Li-ion battery, power bus
│       ├── thermal/           # 8-node nodal lumped thermal network (RK4)
│       ├── ground_station/    # Demodulation, telemetry store & ARQ image reconstruction
│       └── dashboard/         # FastAPI, WebSockets & Plotly visualizers
├── tests/                     # Verification test suites (108 tests)
└── hardware/                  # Schematics and CAD models
```

---

## 6. Verification & Testing

Execute the complete end-to-end test suite across all 8 phases:

```bash
pytest -v
```

All 108 tests pass with zero fabricated data, strict SI units, and complete datasheet traceability.
