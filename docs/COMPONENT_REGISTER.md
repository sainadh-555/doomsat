# GARUDA-1U Traceable Component Register

**Document ID:** GARUDA-REG-001  
**Revision:** 1.0  
**Status:** Approved Baseline  
**System:** GARUDA-1U 1U CubeSat Digital Twin & Mission Simulator  

---

## 1. Classification Methodology

In accordance with GARUDA-1U engineering rules, all physical and operational parameters must be strictly classified:
1. **Datasheet-Derived [D]:** Extracted directly from official manufacturer technical datasheets, application notes, or standardized industry specifications.
2. **Assumed [A]:** Derived from engineering estimates, standard space industry baselines (e.g., ECSS / NASA CubeSat handbook), or geometric calculations pending empirical verification.
3. **Measured [M]:** Empirically verified through laboratory benchtop measurement, calibration rigs, or Hardware-in-the-Loop tests.

---

## 2. Component Register Table

| CID | Subsystem | Component Name | Manufacturer / Part # | Parameters & Ratings | Parameter Source | Datasheet Reference |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **STR-01** | Structure | 1U CubeSat Primary Structure | Custom Aluminum 6061-T6 | Mass: $120.0\text{ g}$<br>Dim: $100 \times 100 \times 113.5\text{ mm}$<br>Yield Strength: $276\text{ MPa}$ | **[D]** Dimensions<br>**[A]** Internal brackets | CubeSat Design Spec Rev 14.1; Al6061-T6 Material Handbook |
| **OBC-01** | OBC | Raspberry Pi Zero 2 W | Raspberry Pi Ltd | Mass: $16.0\text{ g}$<br>Processor: BCM2710A1 Quad Cortex-A53 @ 1.0 GHz<br>RAM: 512 MB LPDDR2<br>Supply: $5.0\text{ V} \pm 5\%$<br>Power: Idle $700\text{ mW}$, Stress $1800\text{ mW}$<br>Op Temp: $-20^\circ\text{C}$ to $+70^\circ\text{C}$ | **[D]** Mass, Power, Temp limits | Raspberry Pi Zero 2 W Product Brief (2021); Broadcom BCM2710A1 Spec |
| **PAY-01** | Payload | 5MP CMOS Camera Sensor | OmniVision OV5647 | Mass: $3.5\text{ g}$ (with M12 lens)<br>Resolution: $2592 \times 1944$<br>Pixel size: $1.4\ \mu\text{m} \times 1.4\ \mu\text{m}$<br>Focal Length: $3.6\text{ mm}$ (FOV $65^\circ$)<br>Interface: MIPI CSI-2 2-lane<br>Supply: $3.3\text{ V} / 1.8\text{ V}$<br>Power: $250\text{ mW}$ active, $20\ \mu\text{W}$ standby | **[D]** Sensor optics & power | OmniVision OV5647 Color CMOS QSXGA Sensor Datasheet Rev 1.2 |
| **COM-01** | TT&C | ESP32 Microcontroller | Espressif ESP32-WROOM-32D | Mass: $4.0\text{ g}$<br>Core: Dual Xtensa LX6 @ 240 MHz<br>SRAM: 520 KB, Flash: 4 MB<br>Supply: $3.0\text{ V} - 3.6\text{ V}$<br>Power: Deep sleep $10\ \mu\text{A}$, Active $240\text{ mW}$ | **[D]** Mass, Voltage, Power profiles | Espressif Systems ESP32-WROOM-32D Datasheet v2.1 |
| **RF-01** | TT&C RF | LoRa RF Transceiver | Semtech SX1276 | Mass: $2.5\text{ g}$<br>Freq Range: $137 - 1020\text{ MHz}$<br>Modulation: LoRa, FSK, OOK<br>Tx Power: $+2\text{ dBm}$ to $+20\text{ dBm}$<br>Rx Current: $10.8\text{ mA}$<br>Tx Current: $120\text{ mA}$ (@ $+20\text{ dBm}$)<br>Sensitivity: $-148\text{ dBm}$ (SF12, 10.4 kHz BW) | **[D]** RF characteristics & sensitivity | Semtech SX1276/77/78/79 Wireless Transceiver Datasheet Rev 7 (2020) |
| **SEN-01** | ADCS | 6-DOF Inertial Measurement Unit | TDK InvenSense MPU6050 | Mass: $2.0\text{ g}$ (breakout)<br>Gyro Range: $\pm 250, 500, 1000, 2000^\circ/\text{s}$<br>Gyro Noise: $0.005^\circ/\text{s}/\sqrt{\text{Hz}}$<br>Accel Range: $\pm 2, 4, 8, 16\ g$<br>Supply: $2.375\text{ V} - 3.46\text{ V}$<br>Power: $13.2\text{ mW}$ active | **[D]** Sensor performance | InvenSense MPU-6000/MPU-6050 Product Specification Rev 3.4 |
| **SEN-02** | ADCS | 3-Axis Digital Magnetometer | STMicroelectronics LIS3MDL | Mass: $1.8\text{ g}$<br>Full-scale: $\pm 4, \pm 8, \pm 12, \pm 16\text{ gauss}$<br>Resolution: $16\text{ bits}$<br>RMS Noise: $3.2\text{ mgauss}$<br>Supply: $1.9\text{ V} - 3.6\text{ V}$<br>Power: $0.9\text{ mW}$ (High-performance mode) | **[D]** Magnetic specs & power | STMicroelectronics LIS3MDL Ultra-low-power 3-axis Magnetometer Datasheet Rev 4 |
| **SEN-03** | ADCS | Silicon PIN Photodiode (x6) | Vishay BPW83 | Mass: $0.25\text{ g}$ each ($1.5\text{ g}$ total)<br>Active Area: $7.5\text{ mm}^2$<br>Spectral Sensitivity: $790 - 1050\text{ nm}$<br>Peak Sensitivity: $950\text{ nm}$<br>Short-circuit current: $I_{sc} \approx 45\ \mu\text{A}$ @ $E_e = 1\text{ mW/cm}^2$<br>Angle of Half Sensitivity: $\phi = \pm 65^\circ$ | **[D]** Optical & electrical parameters | Vishay Semiconductors BPW83 Silicon PIN Photodiode Datasheet Rev 1.9 (2022) |
| **ADC-01** | ADCS | 16-Bit 4-Channel ADC (x2) | Texas Instruments ADS1115 | Mass: $1.5\text{ g}$ each ($3.0\text{ g}$ total)<br>Resolution: 16-bit (860 SPS)<br>Supply: $2.0\text{ V} - 5.5\text{ V}$<br>Power: $0.5\text{ mW}$ operating, $1.5\ \mu\text{W}$ power-down<br>Interface: I2C (4 selectable addresses) | **[D]** ADC resolution & timing | Texas Instruments ADS1113/4/5 Precision ADCs Datasheet SBAS444D |
| **SEN-04** | Health | Environmental Sensor | Bosch Sensortec BME280 | Mass: $1.5\text{ g}$<br>Pressure: $300 - 1100\text{ hPa}$ ($\pm 1\text{ hPa}$)<br>Temp: $-40^\circ\text{C}$ to $+85^\circ\text{C}$ ($\pm 0.5^\circ\text{C}$)<br>Humidity: $0 - 100\%\text{ RH}$ ($\pm 3\%$)<br>Power: $10\ \mu\text{W}$ @ 1 Hz sampling | **[D]** Precision & operating limits | Bosch Sensortec BME280 Combined Sensor Datasheet BST-BME280-DS002-15 |
| **EPS-01** | EPS | 18650 Li-ion Battery Cell | Samsung INR18650-25R / LG | Mass: $45.0\text{ g}$<br>Nominal Voltage: $3.6\text{ V}$<br>Charge Voltage: $4.20\text{ V} \pm 0.05\text{ V}$<br>Cut-off Voltage: $2.50\text{ V}$ (Operational cutoff set to $3.20\text{ V}$)<br>Capacity: $2500\text{ mAh}$ ($9.0\text{ Wh}$)<br>Max Discharge: $20\text{ A}$ continuous | **[D]** Electrical curves & limits<br>**[A]** Degradation rate $5\%/\text{year}$ | Samsung SDI INR18650-25R Specification Data Sheet |
| **EPS-02** | EPS | Body-Mounted Solar Arrays | GaAs Triple-Junction Cells | Mass: $55.0\text{ g}$ total (5 faces active)<br>Cell Efficiency: $\eta = 28.5\%$ @ AM0<br>Effective Area: $70\text{ cm}^2$ per face<br>Peak Power per Face: $2.7\text{ W}$ @ normal incidence | **[A]** Packing factor 0.85<br>**[D]** Triple-junction cell AM0 efficiency | Standard AzurSpace 3G30A Triple Junction Solar Cell Specification |
| **EPS-03** | EPS | Power Management & Regulators | Texas Instruments TPS63020 / Buck-Boost | Mass: $5.0\text{ g}$<br>Input: $1.8\text{ V} - 5.5\text{ V}$<br>Outputs: 3.3V @ 2A, 5.0V @ 1.5A<br>Efficiency: $92\% - 95\%$ | **[D]** Efficiency & quiescent current | Texas Instruments TPS63020 High Efficiency Buck-Boost Converter Datasheet |
| **ANT-01** | TT&C | Monopole / Dipole Deployable Antenna | Spring Steel / Nitinol Tape | Mass: $18.0\text{ g}$<br>Gain: $+2.15\text{ dBi}$ (Quarter-wave)<br>Impedance: $50\ \Omega$<br>VSWR: $\le 1.5:1$ at resonance | **[A]** Gain pattern<br>**[D]** Material elasticity | Amateur Satellite Antenna Design Handbook |

---

## 3. Mass & Power Budget Baseline

- **Total Dry Mass Estimate:**
  - Structure: $120.0\text{ g}$
  - Avionics & Sensors: $32.8\text{ g}$
  - Payload (Camera): $3.5\text{ g}$
  - EPS (Battery, Cells, Regulators): $105.0\text{ g}$
  - Antenna & Harness: $35.0\text{ g}$
  - **Subtotal:** $296.3\text{ g}$
  - **Ballast / Balance Mass:** $450.0\text{ g}$
  - **Total Estimated Mass:** $\approx 746.3\text{ g}$
  - **CubeSat 1U Specification Limit:** $1330.0\text{ g}$
  - **Mass Margin:** $\mathbf{43.8\%}$ (Compliant with REQ-MEC-02).
