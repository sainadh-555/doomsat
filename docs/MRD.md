# GARUDA-1U Mission Requirements Document (MRD)

**Document ID:** GARUDA-MRD-001  
**Revision:** 1.0  
**Status:** Approved Baseline  
**System:** GARUDA-1U 1U CubeSat Mission Simulator & Digital Twin  

---

## 1. Scope & Mission Definition

GARUDA-1U is a 1U CubeSat technology demonstration mission designed to operate in Low Earth Orbit (LEO, circular $500\text{ km}$ altitude, $97.4^\circ$ Sun-Synchronous Orbit). The primary mission objectives are:
1. Low-power Earth observation utilizing an onboard 5MP CMOS camera (OmniVision OV5647).
2. Ground command and telemetry transmission using CSTP (CubeSat Space Transfer Protocol) over Semtech SX1276 LoRa transceivers.
3. Multi-sensor attitude determination using IMU (MPU6050), 3-axis magnetometer (LIS3MDL), and a 6-channel photodiode sun sensor network (BPW83 + ADS1115).
4. Autonomous electrical power and thermal equilibrium management in orbit.

---

## 2. Functional & Performance Requirements Matrix

The requirements cover all 13 core simulation domains:

### Domain 1: Mechanical & Mass Properties
- **REQ-MEC-01 (Form Factor):** The satellite mechanical envelope shall strictly conform to the standard 1U CubeSat specification: $100.0 \pm 0.1\text{ mm} \times 100.0 \pm 0.1\text{ mm} \times 113.5 \pm 0.1\text{ mm}$ (including dispenser rails).
- **REQ-MEC-02 (Mass Limit):** Total launch mass shall not exceed $1.330\text{ kg}$ with a minimum $15\%$ mass growth margin allocated.
- **REQ-MEC-03 (Center of Gravity):** The center of gravity shall reside within a sphere of radius $\le 20.0\text{ mm}$ from the geometric center of the 1U chassis.
- **REQ-MEC-04 (Inertia Tensor):** The full $3 \times 3$ mass moment of inertia tensor $\mathbf{I}$ shall be computed and verified:
  $$I_{ij} = \int (\delta_{ij} r^2 - x_i x_j) \, dm$$

### Domain 2: On-Board Computer (OBC) Subsystem
- **REQ-OBC-01 (Hardware Baseline):** The OBC shall model the Raspberry Pi Zero 2 W architecture (Quad-core ARM Cortex-A53 @ 1.0 GHz, 512 MB LPDDR2).
- **REQ-OBC-02 (Flight Operating States):** The flight software state machine shall support defined modes: `BOOT`, `SAFE_HOLD`, `DETUMBLE`, `NOMINAL_SCIENCE`, `IMAGE_CAPTURE`, `TRANSMIT_TELEMETRY`, `TRANSMIT_IMAGE`, `CRITICAL_LOW_POWER`.
- **REQ-OBC-03 (Watchdog & Fault Management):** Hardware watchdog timers and heartbeat pings between OBC and ESP32 Comms controller shall trigger safe-mode reset if no ping is received for $60.0\text{ s}$.

### Domain 3: Camera Payload Subsystem
- **REQ-PAY-01 (Sensor Format):** The imaging sensor model shall simulate the OmniVision OV5647 5MP CMOS sensor ($2592 \times 1944$ active pixel resolution, $1.4\ \mu\text{m}$ pitch).
- **REQ-PAY-02 (Optical Geometry):** Ground Sampling Distance (GSD) and swath width shall be calculated from orbital altitude $h$ and focal length $f$:
  $$\text{GSD} = \frac{p \cdot h}{f}$$
- **REQ-PAY-03 (Compression & Chunking):** Captured imagery shall be processed, JPEG compressed with selectable quality ($10\% - 95\%$), and partitioned into MTU-compliant CSTP packets (maximum $220\text{ bytes}$ payload per packet).

### Domain 4: TT&C and CSTP Protocol Stack
- **REQ-TTC-01 (Protocol Architecture):** Telemetry, Tracking, and Command shall implement the layered CSTP protocol (Header, Payload, CRC-16-CCITT).
- **REQ-TTC-02 (Framing & Verification):** Packets shall be framed with SLIP boundary delimiters (`0xC0`), sequence numbers, subsystem message IDs, and verified with CRC-16:
  $$P(x) = x^{16} + x^{12} + x^5 + 1 \quad (\text{Polynomial } 0x1021)$$
- **REQ-TTC-03 (Reliable Downlink):** Selective repeat / ARQ acknowledgement mechanism shall support dropped chunk recovery for image files.

### Domain 5: SX1276 RF Transceiver & Physical Layer
- **REQ-RF-01 (Radio Hardware):** Semtech SX1276 LoRa transceiver parameters shall be modeled (Frequency: $436.500\text{ MHz}$ simulated / $865.500\text{ MHz}$ HIL; Bandwidth: $62.5 - 500\text{ kHz}$; Spreading Factor: SF7 - SF12; Coding Rate: 4/5 - 4/8).
- **REQ-RF-02 (Transmit Power):** Configurable RF output power from $+2\text{ dBm}$ to $+20\text{ dBm}$ ($15.8\text{ mW}$ to $100\text{ mW}$).
- **REQ-RF-03 (Receiver Sensitivity):** Sensitivity threshold $S_{rx}$ shall follow:
  $$S_{rx} = -174 + 10\log_{10}(\text{BW}) + \text{NF} + \text{SNR}_{limit} \quad [\text{dBm}]$$

### Domain 6: RF Link Budget & Airtime Simulation
- **REQ-LNK-01 (Path Loss):** Free Space Path Loss (FSPL) shall be dynamically calculated from line-of-sight slant range $d(t)$:
  $$\text{FSPL}(d, f) = 20\log_{10}(d) + 20\log_{10}(f) + 20\log_{10}\left(\frac{4\pi}{c}\right)$$
- **REQ-LNK-02 (Packet Airtime):** Exact LoRa on-air duration $T_{packet}$ shall be calculated from preamble symbols $n_{pre}$, payload symbols $n_{payload}$, and symbol period $T_s = 2^{\text{SF}} / \text{BW}$.
- **REQ-LNK-03 (Margin & BER):** Link margin shall maintain $\ge 6.0\text{ dB}$ at minimum pass elevation angle ($10.0^\circ$).

### Domain 7: Orbital Propagation & Ground Station Visibility
- **REQ-ORB-01 (Propagation Model):** Orbital state shall be propagated using SGP4 with Two-Line Element (TLE) sets and numerical J2 perturbation integrator.
- **REQ-ORB-02 (Coordinate Systems):** Transformations between ECI (J2000), ECEF (WGS-84), Geodetic, and Orbit LVLH shall maintain numerical error $\le 1.0\times 10^{-6}\text{ m}$.
- **REQ-ORB-03 (Ground Station Pass):** Visibility passes over the Ground Station ($17.3850^\circ\text{ N}, 78.4867^\circ\text{ E}$, Hyderabad, India) shall compute elevation, azimuth, slant range, Doppler shift, and eclipse conditions.

### Domain 8: ADCS Dynamics & Attitude Estimation
- **REQ-ADC-01 (Rigid Body Dynamics):** Angular velocity $\boldsymbol{\omega}$ shall propagate via Euler's rotational equations with external disturbance torques:
  $$\mathbf{I}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega}) = \boldsymbol{\tau}_{dist} + \boldsymbol{\tau}_{ctrl}$$
- **REQ-ADC-02 (Kinematics):** Quaternion propagation shall be normalized at each time step:
  $$\dot{\mathbf{q}} = \frac{1}{2} \boldsymbol{\Omega}(\boldsymbol{\omega}) \mathbf{q}$$
- **REQ-ADC-03 (Sensors):** Noise, bias drift, and quantization shall be modeled for MPU6050 (gyro, accel), LIS3MDL (magnetometer), and 6 $\times$ BPW83 photodiodes via ADS1115 16-bit ADCs.
- **REQ-ADC-04 (Attitude Estimation):** Multiplicative Extended Kalman Filter (MEKF) or TRIAD algorithm shall estimate spacecraft orientation within $\le 3.0^\circ$ accuracy.

### Domain 9: Electrical Power Subsystem (EPS)
- **REQ-EPS-01 (Solar Power):** Power generation $P_{in}(t)$ shall account for solar incidence angle $\theta_i$ on 5 active faces and orbital eclipse:
  $$P_{in}(t) = \sum_{k=1}^{6} \max(0, \mathbf{s}_{body} \cdot \hat{\mathbf{n}}_k) \cdot A_k \cdot \eta_{cell} \cdot S_0 \cdot (1 - \text{eclipse})$$
- **REQ-EPS-02 (Battery Model):** 3.7V 2600 mAh Li-ion battery State of Charge (SoC), open-circuit voltage $V_{oc}(\text{SoC})$, and internal resistance $R_{int}$ shall be integrated continuously.
- **REQ-EPS-03 (Power Distribution):** Regulated 3.3V and 5.0V buses shall enforce current limits and low-voltage cutoff ($\text{SoC} < 20\%$).

### Domain 10: Thermal Subsystem
- **REQ-THM-01 (Lumped Parameter Nodes):** Multi-node thermal network (6 external panel faces + internal electronics stack + battery) shall be solved:
  $$m_i c_{p,i} \frac{dT_i}{dt} = Q_{solar,i} + Q_{albedo,i} + Q_{EarthIR,i} + Q_{int,i} - \sum_{j} K_{ij}(T_i - T_j) - \sum_{k} \sigma \epsilon_i F_{ik} A_i (T_i^4 - T_k^4)$$
- **REQ-THM-02 (Operational Temperatures):** Component temperatures shall remain within manufacturer limits:
  - Battery: $0^\circ\text{C}$ to $+45^\circ\text{C}$ (Charge), $-20^\circ\text{C}$ to $+60^\circ\text{C}$ (Discharge)
  - Electronics (RPi Zero 2 W, ESP32): $-20^\circ\text{C}$ to $+70^\circ\text{C}$

### Domain 11: Ground Station & Image Reassembly
- **REQ-GST-01 (Demodulation & Decoding):** Ground station software shall ingest LoRa packets, verify CRC-16, update link statistics, and log raw telemetry.
- **REQ-GST-02 (Image Assembly):** Chunk packets shall be ordered, missing chunks detected, selective repeat NACKs generated, and JPEG image reassembled.

### Domain 12: Mission Control Dashboard
- **REQ-DSH-01 (Architecture):** Real-time web dashboard using FastAPI, WebSockets, and Plotly.
- **REQ-DSH-02 (Displays):** Interactive 3D orbit globe, attitude orientation visualizer, 2D ground track, solar panel illumination, battery SoC gauge, and received photo gallery.

### Domain 13: Hardware-in-the-Loop (HIL) Support
- **REQ-HIL-01 (Serial Transport):** Physical UART bridge (`pyserial`) to communicate with actual ESP32 development boards running embedded CSTP firmware.
- **REQ-HIL-02 (Driver Decoupling):** Architecture shall permit switching between pure Python simulated radio and physical serial hardware driver with identical API interfaces.
