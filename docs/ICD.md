# DOOMSat Interface Control Document (ICD)

**Document ID:** DOOM-ICD-001  
**Revision:** 1.0  
**Status:** Approved Baseline  
**System:** DOOMSat 1U CubeSat Digital Twin & Mission Simulator  

---

## 1. Electrical & Data Architecture Overview

DOOMSat avionics are organized around two primary processing nodes:
1. **On-Board Computer (OBC):** Raspberry Pi Zero 2 W (High-level coordination, camera capture, image processing, filesystem).
2. **TT&C Controller:** Espressif ESP32-WROOM-32D (Low-level real-time radio control, watchdog supervisor, CSTP packet routing, sensor acquisition).

```
+-----------------------------------------------------------------------------------------+
|                                    ELECTRICAL BUSES                                     |
|                                                                                         |
|  +--------------------+                     +--------------------+                      |
|  |   Primary Power    |                     |   Regulated Buses  |                      |
|  |   18650 Li-ion     |                     |   - 3.3V DC Bus    |                      |
|  |   (3.0V - 4.2V)    |                     |   - 5.0V DC Bus    |                      |
|  +---------+----------+                     +---------+----------+                      |
|            |                                          |                                 |
+------------|------------------------------------------|---------------------------------+
             |                                          |
             v                                          v
+------------------------+      UART (115200)   +------------------------+
|  Raspberry Pi Zero 2 W |<====================>|  ESP32-WROOM-32D       |
|  (OBC Node)            |                      |  (TT&C Controller Node)|
+-----------+------------+                      +----+--------+-----+----+
            | MIPI CSI-2                             | SPI    | I2C |    | Analog
            v                                        v        v     v    v
      +------------+                            +--------+  +-----+ +----+
      |   OV5647   |                            | SX1276 |  |I2C  | |BPW |
      | 5MP Camera |                            | LoRa   |  |Sens | |Sun |
      +------------+                            +--------+  +-----+ +----+
```

---

## 2. Subsystem Interface Definitions

### 2.1 OBC <--> TT&C Controller (ESP32) Interface
- **Physical Link:** Asynchronous serial UART (3.3V LVTTL).
- **RPi Pins:** GPIO14 (UART0_TXD, Pin 8), GPIO15 (UART0_RXD, Pin 10), GND (Pin 6).
- **ESP32 Pins:** GPIO16 (UART2_RXD), GPIO17 (UART2_TXD), GND.
- **Baud Rate:** $115,200\text{ bps}$, 8 data bits, no parity, 1 stop bit (8-N-1).
- **Framing Protocol:** SLIP (RFC 1055) framing over CSTP packets.

### 2.2 TT&C Controller (ESP32) <--> SX1276 Transceiver Interface
- **Physical Link:** SPI Bus + Dedicated Interrupt/Control GPIOs.
- **Voltage Level:** 3.3V DC.
- **Pin Mapping:**
  | SX1276 Pin | ESP32 GPIO | Function | Description |
  | :--- | :--- | :--- | :--- |
  | **SCK** | GPIO18 | Output | SPI Clock (up to 10 MHz) |
  | **MISO** | GPIO19 | Input | SPI Master-In-Slave-Out |
  | **MOSI** | GPIO23 | Output | SPI Master-Out-Slave-In |
  | **NSS** | GPIO5 | Output | SPI Chip Select (Active Low) |
  | **DIO0** | GPIO26 | Input | TxDone / RxDone Interrupt |
  | **RESET**| GPIO14 | Output | Hardware Reset (Active Low) |

### 2.3 Sensor Subsystem Bus (I2C Bus 1)
- **Physical Link:** Two-wire I2C bus with $4.7\text{ k}\Omega$ pull-up resistors to 3.3V.
- **Host Controller:** ESP32 (SDA: GPIO21, SCL: GPIO22).
- **Clock Speed:** $100\text{ kHz}$ (Standard Mode) / $400\text{ kHz}$ (Fast Mode).
- **Device Address Table:**
  | Device | Part Number | 7-bit Address | Address Pin Config | Primary Measurement |
  | :--- | :--- | :--- | :--- | :--- |
  | **IMU** | MPU6050 | `0x68` | AD0 = GND | 3-axis $\omega$ ($^\circ/\text{s}$), 3-axis $a$ ($g$) |
  | **Magnetometer** | LIS3MDL | `0x1C` | SDO = GND | 3-axis $B$ ($\text{gauss} / \mu\text{T}$) |
  | **ADC #1** | ADS1115 | `0x48` | ADDR = GND | Photodiodes: +X, -X, +Y, -Y |
  | **ADC #2** | ADS1115 | `0x49` | ADDR = VDD | Photodiodes: +Z, -Z, Batt Volt, Current Sense |
  | **Env Sensor** | BME280 | `0x76` | SDO = GND | Temperature, Pressure, Humidity |

### 2.4 Sun Sensor Network (BPW83 Photodiodes)
- **Sensor Type:** Vishay BPW83 Silicon PIN Photodiodes (spectral range $790 - 1050\text{ nm}$).
- **Configuration:** 6 units mounted orthogonally on each 1U external face (+X, -X, +Y, -Y, +Z, -Z).
- **Signal Conditioning:** Reverse-biased with precision transimpedance amplifier / shunt load resistor producing $0.0 - 3.3\text{ V}$ proportional to cosine of solar incidence angle $\theta$:
  $$V_{diode} = R_{shunt} \cdot I_{sc} \cdot \max(0, \cos\theta)$$
- **ADC Channels (ADS1115):**
  - ADC #1 Channel 0: Sun Sensor +X
  - ADC #1 Channel 1: Sun Sensor -X
  - ADC #1 Channel 2: Sun Sensor +Y
  - ADC #1 Channel 3: Sun Sensor -Y
  - ADC #2 Channel 0: Sun Sensor +Z
  - ADC #2 Channel 1: Sun Sensor -Z
  - ADC #2 Channel 2: Battery Voltage Monitor (divided 2:1)
  - ADC #2 Channel 3: Solar Bus Current Monitor (shunt amplifier)

### 2.5 Camera Payload Interface
- **Physical Link:** 15-pin FFC flat cable carrying 2-lane MIPI CSI-2 + I2C control bus.
- **Sensor:** OmniVision OV5647 5MP CMOS.
- **Interface to RPi:** Dedicated CSI-2 receiver on RPi Zero 2 W VideoCore IV ISP.
- **Control Interface:** Camera control via I2C bus `i2c-10` on RPi.

---

## 3. CSTP (CubeSat Space Transfer Protocol) Specification

### 3.1 Packet Structure
Each CSTP frame is binary encoded:

| Field | Size (Bytes) | Description |
| :--- | :--- | :--- |
| **SLIP_ESC** | 1 | Framing delimiter (`0xC0`) |
| **SYNC** | 2 | Magic bytes: `0xD0 0x01` ("DOOMSat-1") |
| **SRC_ID** | 1 | Source Subsystem / Node ID |
| **DST_ID** | 1 | Destination Node ID |
| **MSG_TYPE** | 1 | Message Type / Opcode |
| **SEQ_NUM** | 2 | 16-bit packet sequence counter (Big-Endian) |
| **PAYLOAD_LEN** | 2 | Length of payload $N \le 220\text{ bytes}$ |
| **PAYLOAD** | $N$ | Variable application payload |
| **CRC16** | 2 | CRC-16-CCITT calculated over header + payload |
| **SLIP_END** | 1 | Frame end delimiter (`0xC0`) |

### 3.2 Subsystem Identifiers (`SRC_ID` / `DST_ID`)
- `0x01`: Ground Station (GS)
- `0x02`: Spacecraft TT&C Controller (ESP32)
- `0x03`: On-Board Computer (RPi Zero 2 W)
- `0x04`: ADCS Subsystem
- `0x05`: Camera Payload Subsystem
- `0x06`: EPS & Power Management

### 3.3 Message Types (`MSG_TYPE`)
- `0x01`: Heartbeat / Ping
- `0x02`: Heartbeat Acknowledge (Pong)
- `0x10`: Housekeeping Telemetry Beacon
- `0x11`: Extended ADCS Telemetry
- `0x12`: Extended EPS & Battery Telemetry
- `0x20`: Image Metadata / Descriptor
- `0x21`: Image Chunk Data
- `0x22`: Image Chunk ACK / Selective NACK
- `0x30`: Uplink Command Request
- `0x31`: Uplink Command Response / Execution Status
