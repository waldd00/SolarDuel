# ☀️ SolarDuel: Dual-Axis PV Tracker vs. Fixed Panel Efficiency Benchmark

<p align="center">
  <img src="https://img.shields.io/badge/Platform-STM32--NUCLEO--G031K8%20%7C%20ESP32-blue?logo=stmicroelectronics"/>
  <img src="https://img.shields.io/badge/Language-C%20%7C%20Python-orange?logo=c"/>
  <img src="https://img.shields.io/badge/IDE-STM32CubeIDE%20%7C%20Arduino-darkblue"/>
  <img src="https://img.shields.io/badge/Sensor-INA226%20%7C%20LDR-green"/>
  <img src="https://img.shields.io/badge/License-MIT-yellow"/>
  <img src="https://img.shields.io/badge/Status-In%20Progress-orange"/>
</p>

> **SolarDuel** is a dual-axis solar tracking system built around a rigorous A/B testing methodology — placing a motor-driven tracker head-to-head against a fixed panel under identical irradiance conditions to produce concrete, data-driven efficiency comparisons.

---

## 📋 Table of Contents

- [Project Objectives](#-project-objectives)
- [System Architecture & BOM](#️-system-architecture--bom)
- [Hardware Schematic](#-hardware-schematic)
- [System Photo](#-system-photo)
- [Control Strategy — The Engineering Core](#️-control-strategy--the-engineering-core)
- [Signal Processing Pipeline](#-signal-processing-pipeline)
- [Repository Structure](#-repository-structure)
- [Efficiency Analysis Results](#-efficiency-analysis-results)
- [How to Run](#-how-to-run)
- [3D Print Settings](#️-3d-print-settings)
- [License](#-license)

---

## 🚀 Project Objectives

- Determine solar azimuth and elevation angle using differential signals from **4 cross-placed LDRs**.
- Drive servo motors via a **multi-layer control stack** (Kalman filter → Moving Average → PID with anti-windup & derivative filter → exponential smoothing) to achieve precise, jitter-free positioning.
- Apply a realistic electrical load using a **47Ω 5W dummy resistor**, forcing the 6V/150mA panels to operate near their maximum power point.
- Collect high-accuracy voltage, current and power data using an **INA226** power monitor managed by an **ESP32** acting as a fully isolated IoT observer.
- Deliver a **live web dashboard** accessible from any device on the same network — no app, no cloud broker required.
- Conduct A/B testing (manual panel swapping under matched conditions) and process the logged data through **MATLAB** for quantitative efficiency analysis.

---

## 🛠️ System Architecture & BOM

The system is intentionally split into two isolated units: the **Tracker (Worker)** and the **Logger (Observer)**. This separation ensures that data acquisition is never affected by motor control timing or interrupt latency.

```
┌─────────────────────────────────────────────────────┐
│                   TRACKER UNIT                      │
│   4× LDR ──ADC──► STM32G031K8 ──PWM──► 2× Servo   │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│                   LOGGER UNIT                       │
│   Solar Panel ──► INA226 ──I2C──► ESP32             │
│                              └──WiFi──► Browser     │
│                              └──HTTP──► CSV Log     │
└─────────────────────────────────────────────────────┘
```

| Component | Role / Specification | Qty |
|---|---|---|
| STM32-NUCLEO-G031K8 | Primary MCU — Motor & LDR control (STM32CubeIDE / HAL) | 1 |
| ESP-WROOM-32 (ESP32) | Secondary MCU — INA226 logging & web server (Arduino IDE) | 1 |
| LDR | Cross-placed light sensors, 3.3V logic with 10kΩ dividers | 4 |
| INA226 | Bi-directional power monitor (I2C, 0.1Ω shunt) | 1 |
| Dummy Load | 47Ω 5W ceramic resistor | 1 |
| Servo Motor | Pan & Tilt actuators | 2 |
| Solar Panel | 6V 150mA monocrystalline | 2 |
| Resistor | 10kΩ LDR voltage dividers | 4 |
| Power Supply | 5V 3A external adapter | 1 |
| Mechanical | Custom 3D printed pan-tilt structure (PLA) | — |

---

## 🔌 Hardware Schematic

The schematic below shows the full wiring for both the Tracker Unit (STM32 + LDRs + Servos) and the Logger Unit (ESP32 + INA226 + dummy load).

[📄 Hardware Schematic (PDF)](Hardware/hardware_schematic.pdf)

> **Key wiring notes:**
> - INA226: `SDA → GPIO21`, `SCL → GPIO22` on the ESP32
> - LDRs: 10kΩ voltage dividers to 3.3V, outputs to STM32 ADC pins
> - Servos powered from the **external 5V adapter** — not from the STM32 3.3V rail
> - 47Ω dummy load connected across the solar panel output terminals

---

## 📷 System Photo

![System Photo](Docs/system_photo.jpeg)

---

## ⚙️ Control Strategy — The Engineering Core

What makes SolarDuel different from a basic LDR tracker is its **multi-layer signal and control pipeline**. Each layer solves a specific real-world problem:

### Layer 1 — Kalman Filter (per LDR channel)
Raw ADC readings from LDRs contain significant high-frequency noise, especially under partially cloudy conditions. A 1D Kalman filter runs independently on each of the 4 channels, separating real light-level changes from sensor noise.

```
Tunable: q (process noise) | r (measurement noise)
Higher r → smoother but slower response
```

### Layer 2 — Moving Average (window = 5)
A 5-sample circular moving average is applied on top of the Kalman output. This dual-stage filtering virtually eliminates transient spikes caused by shadows, cloud edges, or vibration.

### Layer 3 — PID Control with Anti-Windup & Filtered Derivative

The error signal (left−right for pan, top−bottom for tilt) feeds into a full PID controller. Two key improvements over a naive implementation:

**Anti-windup (integral clamping):** When the system is saturated (servo at limit), the integrator is hard-clamped to `±i_limit`. This prevents the integral term from winding up and causing overshoot when the limit is released.

**Filtered derivative:** Raw derivative `(error − prev_error)` is notoriously sensitive to noise. A first-order low-pass filter with tunable `d_alpha` smooths the D-term before it reaches the output, eliminating the high-frequency amplification that causes servo chatter.

```
d_filtered = d_alpha × raw_d + (1 − d_alpha) × d_filtered_prev
```

### Layer 4 — Hysteresis Dead Band
Instead of a single fixed threshold, a two-threshold hysteresis band is used:

```
TOL_ENTER = 80   → motor activates when error exceeds this
TOL_EXIT  = 120  → motor deactivates only when error drops below this
```

This prevents the classic hunting/chattering problem where the servo oscillates around the boundary of a single threshold.

### Layer 5 — Exponential Position Smoothing
Even with a well-tuned PID, abrupt target jumps would cause the servo to snap. The final servo position is driven through a first-order exponential smoother:

```
pos = pos + α × (target − pos)     α = 0.18
```

This produces an organic, inertia-like motion profile — the servo accelerates toward the target and naturally decelerates as it approaches.

### Layer 6 — Float-Preserving PWM Write
A subtle but critical implementation detail: directly casting a `float` PWM value to `uint16_t` truncates rather than rounds, creating invisible 1-step quantization jitter. All PWM writes use:

```c
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(pos_pan + 0.5f));
```

The `+0.5f` offset converts truncation to proper rounding, eliminating the last source of micro-stepping artifacts.

---

## 📡 Signal Processing Pipeline

```
LDR (raw ADC)
     │
     ▼
Kalman Filter ──────── removes high-freq sensor noise
     │
     ▼
Moving Average (N=5) ── removes transient spikes
     │
     ▼
Differential Error ──── (left−right) | (top−bottom)
     │
     ▼
Hysteresis Dead Band ── suppresses hunting near setpoint
     │
     ▼
PID (Anti-Windup + Filtered D) ── calculates correction
     │
     ▼
Exponential Smoother ── organic motion profile
     │
     ▼
Float-Rounded PWM ──── artifact-free servo output
```

---

## 📂 Repository Structure

```
SolarDuel/
├── CAD_Files/
│   └── STL/                        # Print-ready STL files for pan-tilt structure
├── Hardware/
│   └── hardware_schematic.pdf      # Full wiring schematic
├── Docs/
│   ├── dashboard.png               # Web dashboard preview
│   └── system_photo.jpg            # Physical system photo
├── Software_STM32_Control/         # STM32CubeIDE project (C — tracker firmware)
├── Software_ESP32_Logger/          # Arduino IDE project (INA226 + web dashboard)
├── Data_Analysis/
│   ├── tracker_panel_data.csv      # Logged data — tracking panel
│   ├── fixed_panel_data.csv        # Logged data — fixed panel
│   ├── efficiency_analysis.m       # MATLAB analysis script
│   └── efficiency_analysis.png     # Output figure
└── Docs/                           # Block diagrams and project documentation
```

---

## 📊 Efficiency Analysis Results

The MATLAB analysis script (`Data_Analysis/efficiency_analysis.m`) processes both CSV logs and produces a 4-panel comparative figure. Results are based on a 3-hour measurement session (10:00–13:00) under natural sunlight.

### Results Summary

| Metric | Tracker | Fixed Panel |
|---|---|---|
| Peak Efficiency | 74.44% | 35.00% |
| Average Efficiency | 52.31% | 21.14% |
| Peak Power | 670.0 mW | 315.0 mW |
| Average Power | 470.8 mW | 190.2 mW |
| Total Energy Harvested | 1.4125 Wh | 0.5707 Wh |
| **Tracker Energy Gain** | **+147.5%** | — |
| Avg. Instantaneous Gain | 180.3% | — |

### Analysis Figure

![Efficiency Analysis](Data_Analysis/efficiency_analysis.png)

> **(a)** Instantaneous power output over the 3-hour session. The tracker consistently delivers higher power, with both panels showing characteristic cloud-induced dips. **(b)** Panel efficiency relative to the 0.9W rated maximum. **(c)** Cumulative energy harvested — the tracker collects 1.41 Wh vs. 0.57 Wh for the fixed panel over the same period. **(d)** Instantaneous tracker gain over the fixed panel; the average gain of ~180% highlights the advantage of continuous sun-facing orientation.

---

## 🧑‍💻 How to Run

**1 — Tracker (STM32)**
- Open `Software_STM32_Control` in STM32CubeIDE.
- Compile and flash to the NUCLEO-G031K8.
- Power LDRs from 3.3V, servos from the external 5V adapter.
- On first boot, the system performs a **soft-start** routine — servos ramp smoothly to the home position over 1 second instead of snapping.

**2 — Logger (ESP32)**
- Open `Software_ESP32_Logger` in Arduino IDE.
- Install **ESP Async WebServer** by *mathieucarbou* from Library Manager.
- Set your WiFi credentials in the sketch.
- Flash to the ESP32.
- Wire the INA226: `SDA → GPIO21`, `SCL → GPIO22`.
- Connect the target panel (tracker or fixed) to the INA226 input, with the 47Ω dummy load to ground.
- Open Serial Monitor (115200 baud) — the assigned IP address will be printed.
- Navigate to `http://<IP>` from any browser on the same network.

**3 — Data Collection**
- Use the **Start Recording** button on the dashboard to begin logging.
- Click **Stop & Download CSV** to export the session as a timestamped CSV file.
- Swap panels (A/B testing) and repeat to collect matched irradiance data for both configurations.

**4 — Analysis**
- Place both CSV files and `efficiency_analysis.m` in the same folder.
- Run the script in MATLAB — the summary table prints to the console and the figure saves as `efficiency_analysis.png`.

---

## 🖨️ 3D Print Settings

All mechanical parts in `CAD_Files/STL/` were printed with the following settings:

| Parameter | Value |
|---|---|
| Printer | Creality CR-10 Smart Pro |
| Slicer | Creality Slicer |
| Material | PLA |
| Infill | 15% |
| Supports | Depends on part orientation — check individual STL |

> **Attribution:** The STL files in `CAD_Files/STL/` are sourced from [this Instructables project](https://www.instructables.com/SOLAR-TRACKER-TILTPAN-PANEL-FRAME-LDR-MOUNTS-RIG/) and are not original work. All credit goes to the original author. Files are shared here under the terms of the original license for non-commercial, educational use only.

---

## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

*SolarDuel is an applied engineering study at the intersection of automatic control theory, embedded systems design, and data-driven performance analysis.*
