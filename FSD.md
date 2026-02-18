# FSD.md — IoT LoRa Sensor Node (3-Node TDMA, NTP-Based Scheduling, SDR Test)

## 1. Purpose

Develop firmware for **three identical microcontroller-based IoT nodes** that:
- measure **soil moisture**, **soil temperature**, **battery input voltage**, and **ambient temperature & humidity**
- transmit measurements via **LoRa** in **non-overlapping time slots** (TDMA-like)
- derive their slot timing autonomously using **UTC time fetched from an NTP server**
- are validated using an **SDR receiver connected to SDRAngel** that decodes LoRa frames for test purposes

The gateway is treated as an **SDR observer only** in this stage. There is no LoRaWAN stack and no downlink control requirement.

---

## 2. Scope

### In scope
- Sensor acquisition from the connected sensors:
  - **SEN0308** (soil moisture, analog)
  - **SEN0385** (ambient temperature & humidity, I²C)
  - **DFR0198** (DS18B20-based soil temperature, 1-Wire)
- Battery input voltage measurement via ADC + divider
- NTP-based time sync (daily)
- Slot-based LoRa TX schedule for 3 nodes
- Payload formatting for SDR decode
- Serial logs for development and test validation

### Out of scope (for this stage)
- LoRaWAN (join, uplink/downlink, network server)
- Gateway logic beyond SDR decoding
- OTA updates
- Full power optimization and deep-sleep optimization (can be added later)
- Long-term calibration models for sensors (only basic conversions)

---

## 3. Hardware Target and Environment

### 3.1 Board
- Target board: **Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)**
- Firmware developed using **Arduino CLI** toolchain

### 3.2 Node identities
There are **three nodes** with compile-time identities:
- Node1: `NODE_ID=1`
- Node2: `NODE_ID=2`
- Node3: `NODE_ID=3`

All nodes share the same codebase; `NODE_ID` changes at compile time.

---

## 4. Wiring Constraints (Must Match)

### 4.1 Mandatory pin mapping
- **DFR0198 (DS18B20 1-Wire DQ)** → GPIO **4**
- **SEN0308 analog output** → GPIO **7**
- **Battery divider node (Vbat_in)** → GPIO **2**
- **I²C SCL** → GPIO **42**
- **I²C SDA** → GPIO **41**

- **S_PWR_CNTRL (sensor 3V3 enable, active-low P-MOSFET gate)** -> GPIO **47**

### 4.2 Battery divider
Divider values:
- R1 = **470 kΩ** (VBat → R1 → Vbat_in)
- R2 = **100 kΩ** (Vbat_in → R2 → GND)

Formula:
- `Vbat_in = Vbat * (R2 / (R1 + R2)) = Vbat * (100k / 570k) ≈ Vbat * 0.1754386`
- `Vbat ≈ Vbat_in * 5.7` (nominal)
- Calibrated scale in firmware: `Vbat ~= Vbat_in * 6.05`

---

## 5. Sensors and Interfaces

### 5.1 SEN0308 - Soil Moisture (Analog)
- Interface: ADC read on GPIO7
- Output: voltage (V)
- Requirements:
  - Provide computed voltage in volts

### 5.2 SEN0385 - Ambient Temperature & Humidity (I2C)
- Interface: I²C on SDA=GPIO41, SCL=GPIO42
- Output: ambient temperature (°C) and humidity (%RH)
- Requirements:
  - Detect sensor presence at boot and report status
  - Provide temp and humidity in every measurement message if sensor is present

### 5.3 DFR0198 (DS18B20) — Soil Temperature (1-Wire)
- Interface: 1-Wire on GPIO4 with pull-up resistor between VCC and DQ. 
- Output: soil temperature (°C)
- Requirements:
  - Detect sensor presence at boot and report status
  - Provide temperature in every measurement message if present

### 5.4 Battery Voltage (Analog)
- Interface: ADC read on GPIO2 (divider node)
- Output: Vbat in volts
- Requirements:
  - Provide computed Vbat (float with 2 decimals in payload)

---

## 6. Time Synchronization

### 6.1 Time base
- Time base is **UTC**
- Nodes fetch UTC from an **NTP server** via Wi-Fi

### 6.2 NTP fetch frequency
- Each node performs NTP sync:
  - **once per day** (every 24 hours)
  - on boot, NTP is fetched immediately
- Between sync events, nodes keep time via the MCU clock and calculate elapsed time since last sync.

### 6.3 NTP requirements
- NTP server: `pool.ntp.org`
- Time validity check:
  - epoch must exceed a configured threshold (e.g., `>= 1700000000`)
- Behavior if Wi-Fi/NTP fails:
  - Node does not transmit scheduled measurements until time becomes valid
  - Node retries Wi-Fi/NTP periodically (configurable, default: every 5 minutes)

---

## 7. LoRa Configuration (TX-only, SDR-friendly)

### 7.1 Purpose
LoRa frames must be decodable by SDR (SDRAngel + ChirpChat) reliably.

### 7.2 PHY parameters (initial test configuration)
- Frequency: **868.100 MHz**
- Bandwidth: **125 kHz**
- Spreading factor: **SF12**
- Coding rate: **4/5**
- Preamble length: **8 symbols**
- TX power: **14 dBm**
- Explicit header: **ON**
- CRC: **ON**
- IQ inversion: **OFF**

### 7.3 Stack/API requirement
Use Heltec radio stack:
- include `LoRaWan_APP.h`
- use `Radio.*` API
- call `Radio.IrqProcess()` in the main loop

---

## 8. Slot Scheduling (3 Nodes, 10-Minute Frames)

### 8.1 Scheduling goal
Avoid collisions by assigning **distinct minute-slots** per node. Each node calculates the current UTC minute and determines whether it is allowed to transmit.

### 8.2 Frame definition
Time is partitioned into **10-minute frames** aligned to UTC:
- frame boundaries at minutes: **00, 10, 20, 30, 40, 50**

### 8.3 Slot minutes per node
Within each 10-minute frame:

- Node1 (`NODE_ID=1`) transmits at minutes **1** and **5**
  - 01,05, 11,15, 21,25, 31,35, 41,45, 51,55
- Node2 (`NODE_ID=2`) transmits at minutes **2** and **6**
  - 02,06, 12,16, 22,26, 32,36, 42,46, 52,56
- Node3 (`NODE_ID=3`) transmits at minutes **3** and **7**
  - 03,07, 13,17, 23,27, 33,37, 43,47, 53,57

### 8.4 Transmission window
To reduce drift overlap, transmission is permitted only in:
- seconds **0..10** of the slot minute

### 8.5 Burst policy
In each eligible slot:
- send a burst of **3 frames**
- `SEQ=0..2`
- inter-frame gap: **min 1200 ms** (500 ms requested, minimum enforced)

### 8.6 Anti-repeat guard
Within the same slot minute:
- transmit **at most one burst**
- the firmware must track a slot-identifier (e.g., hour+minute) to prevent repeated triggering.

---

## 9. Measurement Cycle

### 9.1 Measurement timing
- Measurements are taken immediately before each slot transmission burst.
- All values included in every payload (if sensors are available).

### 9.2 Data fields
- Node identity: `NODE`
- Time: `TIME` (UTC epoch seconds)
- Sequence: `SEQ` (0..2)
- Soil moisture:
  - transmitted gravimetric estimate `SM_PCT` (%), derived from calibrated `SM_V` by linear regression
  - `SM_V` remains available in serial diagnostics but is not sent in the LoRa payload
- Soil temperature:
  - `ST_C` (°C)
- Ambient:
  - `AT_C` (°C)
  - `AH_PCT` (%RH)
- Battery:
  - computed volts `VB_V`

---

## 10. Payload Format (ASCII for SDR Test)

### 10.1 Rationale
ASCII payloads simplify SDR validation and debugging.

### 10.2 Payload structure
Each transmitted frame shall contain a single ASCII line:

NODE=3 TIME=1768902183 SEQ=2 SM_PCT=46 ST_C=18.75 AT_C=21.40 AH_PCT=46.00 VB_V=11.92

### 10.3 Future Optimization (Binary / Fixed-Point Payload)
For field deployment, consider switching to a compact binary payload with fixed-point scaling to reduce airtime.
Example schema (target ~12-14 bytes):
- NODE (uint8)
- SEQ (uint8)
- TIME (uint32, epoch)
- SM_PCT (uint8, 0..100)
- ST_C (int16, C * 100)
- AT_C (int16, C * 100)
- AH_PCT (uint8, % * 2)
- VB_V (uint16, mV)
Receiver reconstructs values using the inverse scale factors.

## 11. Serial Logging (Development Support)

### 11.1 Required logs
- On boot:
  - `System alive, Node ID = X`
  - `WiFi OK, IP=...` or `WiFi FAIL`
  - `NTP OK, epoch=...` or `NTP FAIL`
  - `LoRa init OK`
  - sensor detection status (present/missing)
- Periodic debug (at least every ~2 seconds):
  - `UTC HH:MM:SS next_slot=<minute>`
- At TX:
  - `TX <payload line>`

---

## 12. Acceptance Criteria

A node firmware build is accepted when:

1) **Pin constraints** are met exactly as specified in section 4.
2) On boot with valid Wi-Fi:
   - the node obtains NTP time and logs `NTP OK`.
3) Scheduler correctness:
   - Node1 only transmits in minutes 1 and 5 of each 10-minute frame, within seconds 0..10.
   - Node2 only transmits in minutes 2 and 6, within seconds 0..10.
   - Node3 only transmits in minutes 3 and 7, within seconds 0..10.
4) Burst correctness:
   - Each slot produces exactly 3 frames with SEQ=0..2 and ~500 ms gaps.
5) Payload correctness:
   - SDR decoding in SDRAngel yields ASCII lines matching the payload format.
   - `TIME` is a valid epoch and progresses correctly.
6) Sensors:
   - Each transmitted payload includes all measurement fields.
7) NTP refresh:
   - Node attempts NTP resync at least once per 24 hours.

---

## 13. Non-Functional Requirements

- Firmware shall not crash or reboot during normal operation.
- LoRa transmissions shall remain within the defined slot windows.
- Code shall be readable and modular (separate helper functions for sensors, time, scheduling, and TX).

---

## 14. Build Notes (for multi-node programming)

- Same source code for all nodes.
- `NODE_ID` is defined at compile time (e.g., `-DNODE_ID=1`).
- Each node build uses a distinct build path (e.g., `.build/node1`, `.build/node2`, `.build/node3`) to avoid artifact collisions.

---

## 15. Implemented Calibration Updates and Rationale

Version note:
- Effective date: `2026-02-13`
- Calibration revision: `CAL-2026-02-13-A`
- Applies to firmware paths: `src/main.cpp` and `src/maintest.cpp`

### 15.1 Battery voltage calibration changes
- Firmware uses:
  - `VB_V = ((VB_RAW / 4095) * kAdcVref) * kVbatScale * kVbatFineTrim`
- `kVbatScale` remains the divider calibration scale.
- `kVbatFineTrim` is now per-node:
  - Node1: `1.0431`
  - Node2: `1.0508`
  - Node3: `1.0067`

Rationale:
- Field measurements showed node-to-node battery voltage bias after ADC conversion.
- Per-node trim compensates board-level/channel-level ADC and analog-front-end variation while preserving a common formula.

### 15.2 Soil moisture voltage calibration changes
- Soil moisture voltage is now:
  - `SM_V = ((SM_RAW / 4095) * kAdcVref) * kSmFineTrim`
- `kSmFineTrim` is per-node:
  - Node1: `0.9941`
  - Node2: `0.9969`
  - Node3: `0.9818`

Rationale:
- Node-level offsets were observed between measured probe voltage and printed `SM_V`.
- Per-node trim improves agreement of reported voltage with measured voltage.

### 15.3 `SM_PCT` moved to gravimetric regression model
- Legacy raw-threshold mapping (`kSoilMoistureMin/kSoilMoistureMax`) was replaced.
- `SM_PCT` is now computed from calibrated `SM_V` using pooled gravimetric OLS fit:
  - `SM_PCT = clamp(114.608 - 43.252 * SM_V, 0, 100)`
- `SM_PCT` is rounded to nearest integer for transmission/logging.

Rationale:
- Gravimetric calibration data from multiple sensors and moisture classes provides a physically meaningful basis for `%` moisture estimation.
- Voltage-to-gravimetric mapping is more defensible scientifically than raw ADC-threshold interpolation.

### 15.4 Reference calibration note
- Full regression discussion, dataset table, and implementation snippet are documented in:
  - `MOISTURE_GRAVIMETRIC_REGRESSION.md`

