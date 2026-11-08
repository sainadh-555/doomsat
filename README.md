# DOOMSat: 1U CubeSat Digital Twin & Mission Simulation

[![Python 3.11](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/downloads/release/python-3119/)
[![Architecture](https://img.shields.io/badge/architecture-1U%20CubeSat-orange.svg)]()
[![Status](https://img.shields.io/badge/development-Phase%201%20Foundation-green.svg)]()

DOOMSat is an engineering-grade digital twin and multi-physics mission simulator for a 1U CubeSat ($100 \times 100 \times 100\text{ mm}$), developed with high-fidelity aerospace and embedded systems modeling in Python.

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

- [x] **Phase 1: Architecture, Requirements, Repository Foundation & Configuration**
- [ ] **Phase 2: Mechanical Assembly and Mass Properties**
- [ ] **Phase 3: OBC and Camera Simulation**
- [ ] **Phase 4: ESP32, SX1276, CSTP and RF Communication**
- [ ] **Phase 5: Orbit and ADCS**
- [ ] **Phase 6: EPS and Thermal**
- [ ] **Phase 7: Ground Station and Mission Dashboard**
- [ ] **Phase 8: Complete System Integration and Hardware-in-the-Loop**

---

## 4. Repository Structure

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
│       ├── sim/               # Discrete-event simulation engine
│       ├── mechanical/        # Mass budget & inertia tensor
│       ├── obc/               # RPi flight software model
│       ├── payload/           # OV5647 camera & image processor
│       ├── comms/             # CSTP protocol & SX1276 LoRa RF channel
│       ├── orbit/             # SGP4 & Keplerian orbital propagator
│       ├── adcs/              # Attitude dynamics & sensor models
│       ├── eps/               # Solar power & battery storage model
│       ├── thermal/           # Multi-node lumped parameter thermal model
│       ├── ground_station/    # Demodulation & image reconstruction
│       └── dashboard/         # FastAPI / WebSocket mission control
├── tests/                     # Verification test suites
└── hardware/                  # Schematics and CAD models
```

---

## 5. Verification & Testing

To execute the test suite:

```bash
pytest -v
```
