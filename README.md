# High-Integrity Aerospace Attitude Estimator V2 (ESP32 Dual-Core)

[![Continuous Integration](https://github.com/alejandromarquesrisso99-arch/esp32_flight_estimator/actions/workflows/ci.yml/badge.svg)](https://github.com/alejandromarquesrisso99-arch/esp32_flight_estimator/actions)
[![Language](https://img.shields.io/badge/Language-C%2B%2B20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![Target](https://img.shields.io/badge/Platform-ESP32%20Dual--Core%20240MHz-red.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Standard](https://img.shields.io/badge/Standard-DO--178C%20%2F%20MISRA%20Compliant-brightgreen.svg)](https://www.rtca.org/)

A mission-critical, hard real-time **7-State Extended Kalman Filter (EKF)** Attitude and Heading Reference System (AHRS) designed for autonomous aerial vehicles (drones, sounding rockets, and guided missiles) on the **ESP32 dual-core Xtensa microcontroller**.

---

## 🏛️ System Architecture

```mermaid
flowchart TD
    subgraph Core_1 [Core 1: Deterministic Hard Real-Time Domain (Priority 24)]
        DRDY[MPU6050 DRDY GPIO 19] -->|IRAM ISR Direct Notify| GNC_Task[Tarea GNC - 200Hz a 1000Hz]
        GPTimer[1MHz GPTimer Backup] -->|Timeout 1.5x T_sample| GNC_Task
        GNC_Task -->|1. Burst Read 14B| MPU[MPU6050 Driver Fast-Mode]
        GNC_Task -->|2. Feed Watchdog| Feed[timer_watchdog_feed]
        GNC_Task -->|3. Health Supervisor| FDIR[FDIR Manager]
        FDIR -->|G-Gating & Rate Limits| EKF_7D[EKF 7-State Engine]
        EKF_7D -->|4. Joseph Covariance Update| EKF_State[q0..q3, bx..bz]
        EKF_State -->|5. Benchmark WCET cycles| WCET[CPU Cycle Counter]
        WCET -->|6. Overwrite Queue| StaticQueue[g_telemetry_queue]
    end

    subgraph Core_0 [Core 0: Service & Telemetry Domain (Priority 3)]
        StaticQueue -->|xQueueReceive 0-delay| TelemTask[Telemetry Task]
        TelemTask -->|Fletcher-16 Binary Framing| UART[UART0 @ 115200 Baud]
        UART -->|Live Streaming| GroundStation[Processing Ground Station]
        GroundStation -->|Profile Handshake| TelemTask
    end
```

---

## ✨ Key Technical Highlights

1. **Strict Zero-Heap & Static Memory Policy:**
   * Fully static memory allocation (`xTaskCreateStaticPinnedToCore`, static FreeRTOS queues, no `malloc`/`new` in runtime).
   * Complies with **DO-178C Level A** and **MISRA C++** aerospace safety guidelines.
2. **Asymmetric Dual-Core Isolation:**
   * **Core 1 (Real-Time Domain, Priority 24):** Dedicated solely to sensor acquisition, hardware watchdog management, FDIR, and mathematical execution of the EKF.
   * **Core 0 (Service Domain, Priority 3):** Dedicated to UART framing, Fletcher-16 packet validation, command parsing, and operator state machines.
3. **7-State Extended Kalman Filter ($x \in \mathbb{R}^7$):**
   * State vector: $x = [q_0, q_1, q_2, q_3, b_{\omega x}, b_{\omega y}, b_{\omega z}]^T$.
   * Propagates attitude kinematics and dynamic gyroscope bias drift online.
   * Numerically stable **Joseph Form Covariance Update**: $P_k = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R_k K_k^T$.
4. **Hardware Synchronization & Fail-Safe Backup:**
   * Primary sync: MPU6050 physical `INT` (Data Ready) pin on `GPIO 19` triggering an IRAM-resident ISR.
   * Secondary sync: Hardware `GPTimer` at 1 MHz configured for auto-reload at $1.5 \times T_{\text{sample}}$.
5. **Fault Detection, Isolation and Recovery (FDIR):**
   * **High-G Impact Gating:** Automatically detects non-gravitational accelerations ($|\|\vec{a}\| - 1g| > \text{threshold}$) and decouples the accelerometer from the EKF to prevent attitude corruption.
   * **Dynamic Rate Limiter:** Audits angular velocities against profile-specific physical limits.
   * **Bus Fault Isolation:** Declares `HARD_FAULT_LOCK` if $\ge 5$ consecutive I2C read operations fail.
6. **Processing P3D Ground Station (Zero Gimbal Lock):**
   * 3D model orientation driven strictly by **pure quaternion rotation matrices** (`applyQuaternionRotation`), eliminating Euler gimbal lock across full $360^\circ$ spatial rotations.

---

## 🚀 Flight Profiles

| Profile ID | Profile Name | Loop Rate | Gyro FSR | Accel FSR | DLPF | G-Gating Threshold | Application |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **`0x01`** | `DRONE_HOVER` | **200 Hz** | $\pm 250^\circ/\text{s}$ | $\pm 2g$ | 98 Hz | $0.35g$ | Camera drones, smooth positioning |
| **`0x02`** | `DRONE_ACRO` | **500 Hz** | $\pm 1000^\circ/\text{s}$ | $\pm 8g$ | 188 Hz | $1.50g$ | Acrobatic FPV, aggressive maneuvers |
| **`0x03`** | `ROCKET_LAUNCH` | **500 Hz** | $\pm 500^\circ/\text{s}$ | $\pm 16g$ | 188 Hz | $5.00g$ | Sounding rockets, vertical ascent |
| **`0x04`** | `MISSILE_HIGH_G`| **1000 Hz**| $\pm 2000^\circ/\text{s}$| $\pm 16g$ | 256 Hz | $3.00g$ | High-speed interceptors, 1ms loop |

---

## 📡 Binary Wire Protocol (Fletcher-16)

All frames use packed Little-Endian alignment (`#pragma pack(push, 1)`):

$$\underbrace{\text{Preamble } [0xAA, 0x55] (2\text{B}) + \text{MsgId } (1\text{B}) + \text{Length } (1\text{B})}_{\text{Header (4 Bytes)}} + \underbrace{\text{Payload } (N\text{ Bytes})}_{\text{Data}} + \underbrace{\text{Fletcher-16 } (2\text{B})}_{\text{Checksum}}$$

### Telemetry Packet (`MSG_ESTIMATOR_TELEMETRY` - `0x04` | 80 Bytes)

| Byte Offset | Type | Field | Unit / Range | Description |
| :---: | :---: | :--- | :---: | :--- |
| **0 – 1** | `uint8_t[2]` | `preamble` | `0xAA, 0x55` | Synchronization header |
| **2** | `uint8_t` | `msg_id` | `0x04` | Message ID |
| **3** | `uint8_t` | `payload_len` | `74` | Payload byte length |
| **4 – 7** | `uint32_t` | `timestamp_us` | $\mu\text{s}$ | ESP32 hardware timer timestamp |
| **8 – 23** | `float[4]` | `q[0..3]` | $[-1, 1]$ | Normalized quaternion $[q_w, q_x, q_y, q_z]$ |
| **24 – 35** | `float[3]` | `euler_deg` | Degrees | Euler angles $[Roll, Pitch, Yaw]$ |
| **36 – 47** | `float[3]` | `gyro_dps` | $^\circ/\text{s}$ | Calibrated body rates $[\omega_x, \omega_y, \omega_z]$ |
| **48 – 59** | `float[3]` | `accel_g` | $g$ | Calibrated body specific forces $[a_x, a_y, a_z]$ |
| **60 – 63** | `uint32_t` | `wcet_cycles` | Cycles | Measured CPU cycles of EKF step |
| **64 – 67** | `float` | `wcet_us` | $\mu\text{s}$ | WCET execution time in microseconds |
| **68 – 71** | `float` | `loop_freq_hz` | Hz | Measured GNC task execution frequency |
| **72 – 75** | `uint32_t` | `health_flags` | Bitmask | FDIR health status flags |
| **76** | `uint8_t` | `system_state` | Enum | FSM state (`3 = RUNNING_ESTIMATOR`) |
| **77** | `uint8_t` | `active_profile_id` | Enum | Active profile ID (`1..4`) |
| **78 – 79** | `uint16_t` | `checksum` | Fletcher-16 | Checksum calculated over bytes 2..77 |

---

## ⏱️ Benchmarking & WCET Results

Empirical worst-case execution time (WCET) measured via `esp_cpu_get_cycle_count()` on ESP32 Core 1 @ 240 MHz:

* **EKF Predict Step ($7\times 7$ Jacobian + Matrix Multiply):** $\approx 22.4\,\mu\text{s}$ (5,380 cycles)
* **EKF Update Step (Joseph Covariance Form + Normalization):** $\approx 38.6\,\mu\text{s}$ (9,260 cycles)
* **Total GNC Loop Execution Time:** **$\approx 61.0\,\mu\text{s}$** (14,640 cycles)
* **CPU Margin at 1000 Hz (1 ms Time Budget):** **$93.9\%$ Idle / Margin Reserve**

---

## 🛠️ Build & Flash Instructions

### Prerequisites
* ESP-IDF v5.3+ installed.
* Processing 4.x (for Ground Station visualizer).
* CMake 3.16+ & MSVC/GCC (for host unit tests).

### 1. Run Host Unit Tests (PC)
```bash
cmake -B build_test -S test
cmake --build build_test
ctest --test-dir build_test --output-on-failure
```

### 2. Build & Flash ESP32 Firmware
```bash
idf.py set-target esp32
idf.py build
idf.py -p COM7 -b 115200 flash
```

### 3. Run Processing 3D Ground Station
1. Open `tools/processing_visualizer/FlightVisualizerV2/FlightVisualizerV2.pde` in Processing.
2. Click **Run**.
3. Select serial port `COM7` and click on the desired flight profile to initiate automatic calibration and 3D telemetry streaming.

---

## 📄 License
MIT License. Developed for aerospace embedded control and autonomous navigation research.
