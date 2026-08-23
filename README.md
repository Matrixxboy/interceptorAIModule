# Vision-Guided Autonomous Drone Companion System

PC vision lock for an **INAV/ArduPilot FPV drone**: camera tracks a target, then visual servoing steers **yaw + pitch** to keep it centered. The system is currently implemented as a Python-based PC prototype, with a detailed hardware and software specification targeting a production-grade Jetson Orin Nano + STM32 embedded system.

> **Safety:** Bench with **props off** first. This sends real RC channel values to the flight controller.

---

## 1. Current Prototype (PC / Python)

The current implementation is a PC-based prototype that communicates with the flight controller via MSP over USB/Serial.

### What it does

| Stage | Behavior |
|--------|----------|
| **Lock** | Mouse-drag ROI (`L`) or YOLO auto-lock (`Y`) |
| **Track** | CSRT between frames + YOLO refresh / reacquire |
| **Follow** | FPV visual servo → yaw & pitch sticks toward target center |
| **FC** | Continuous MSP AETR + ARM (CH5) + flight mode (CH6) |

### Hardware & INAV setup

| Item | Notes |
|------|--------|
| FPV / USB camera | Seen by Windows as a camera index |
| Flight controller | INAV with MSP on a UART |
| USB–serial link | e.g. FTDI / onboard USB VCP → `COMx` |
| Channel map | **AETR** (default in `main.py`) |

#### INAV Modes tab (typical)
| Mode | Channel | Range |
|------|---------|--------|
| **ARM** | CH5 / AUX1 | high ≈ 1800 |
| **ANGLE** | CH6 / AUX2 | high ≈ 1900 |

### Install

```bash
cd inercepterAI
python -m venv .venv

# Windows
.venv\Scripts\activate

pip install -r requirements.txt
```

*(Optional)* For CUDA (much faster YOLO):
```bash
pip uninstall -y torch torchvision
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
```

### Run

#### Calibrate first (recommended, Props off!)
```bash
python calibration_fpv.py
```
1. Connect FC → set COM port → ANGLE ON.
2. Hold Yaw LEFT/RIGHT and Pitch UP/DOWN — craft should move that way.
3. Open camera → drag-lock a target.
4. Enable LIVE FOLLOW. If it turns the wrong way, flip Yaw / Pitch.
5. Save JSON → `calibration.json`.

#### Fly / Bench Follow
```bash
python main.py
```

### Controls

| Key | Action |
|-----|--------|
| **L** | Start lock selection (drag box) |
| **Y** | YOLO auto-lock |
| **E** / **D** | Enable / Disable follow assist |
| **A** / **X** | Arm (CH5) + Mode ON / Disarm + Mode OFF |
| **M** | Toggle flight mode CH6 |
| **0** | Force CH6 = 1900 |
| **Q** | Quit |

---

## 2. Onboard Radxa ZERO 3 (RK3566, 2GB RAM) Companion Setup

To run the tracking daemon on the **Radxa ZERO 3** (2GB RAM Linux SBC) connected to an INAV / Betaflight FC via MSP over UART2 (`/dev/ttyS2`):

### Installation & System Setup
```bash
# 1. Run automated Radxa setup (Installs OpenCV, V4L2, GStreamer, sets user permissions & performance governor)
chmod +x scripts/setup_radxa.sh
./scripts/setup_radxa.sh

# 2. Run system diagnostics
python3 scripts/diagnose_system.py
```

### Model Conversion (On PC)
On 2GB RAM Linux SBCs, PyTorch causes Out-Of-Memory (OOM) crashes. OpenCV DNN with ONNX models is used instead.
Convert your PyTorch `.pt` model to `.onnx` on your PC before copying to Radxa:
```bash
python scripts/export_to_onnx.py --weights models/drone_missile_best.pt
```

### Running Onboard
```bash
# Start tracking daemon manually
python3 main.py --config config.json

# Enable auto-start on boot via systemd
sudo cp interceptor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable interceptor
sudo systemctl start interceptor
```

---

## 2. Production-Grade Embedded System Architecture & Technical Specification

> [!NOTE]
> **Target Platform:** NVIDIA Jetson Orin Nano (8GB) + STM32H753 MCU  
> **Primary AI Core:** YOLOv8-Nano TensorRT INT8 (>60 FPS) | **Control Frequency:** 8 kHz Inner Loop / 400 Hz EKF3 / 50 Hz SBC Setpoint Stream

This section specifies the end-to-end embedded software and hardware architecture for the production version of the system. Designed as a high-performance, modular, and fail-safe hardware extension, this system mounts onto multirotor platforms to provide real-time target tracking, optical distance estimation, autonomous velocity setpoint generation, and immediate pilot override capability.

The system features a dual-processor heterogeneity model:
1. **Companion Computer (High-Level AI/Vision Core):** NVIDIA Jetson Orin Nano running real-time target detection (YOLOv8-Nano TensorRT INT8), visual object tracking (ByteTrack), monocular distance estimation, and kinematic PID vector computation.
2. **Flight Controller Core (Real-Time Safety/Control Core):** STM32H753 running ArduPilot/PX4 RTOS, handling sensor fusion (EKF3), attitude control loops, motor PWM output, manual RC receiver decoding, and hardware-level override arbitration.

> [!IMPORTANT]
> **Safety Guarantees**: Direct manual RC stick deflection (>10%) or heartbeat loss (>500 ms) automatically revokes autonomous authority, falling back to manual angle control or auto-hover.

### System Architecture Diagram

```mermaid
flowchart TB
    subgraph Power ["POWER DISTRIBUTION & ISOLATION"]
        Battery["4S-6S LiPo Battery\n(14.8V - 25.2V)"] --> PDB["Main PDB / Current Sensor\n(Volt / Amp Telemetry)"]
        PDB --> BEC["Dual Buck Regulator (BEC)\n5.1V @ 5A (FC) | 9.0V @ 3A (SBC/CAM)"]
    end

    subgraph Ground ["PILOT & GROUND CONTROLS"]
        Transmitter["Pilot Transmitter\n(ExpressLRS / CRSF 2.4GHz)"]
    end

    subgraph SBC ["COMPANION COMPUTER (NVIDIA Jetson Orin Nano)"]
        direction TB
        Camera["Sony IMX477 CSI Cam\n(4-Lane MIPI CSI-2)"] --> Preproc["Image Acquisition & Preprocessing\n(V4L2 DMA / NVMM / CUDA Zero-Copy)"]
        Preproc --> VisionEngine["Inference & Computer Vision\n(YOLOv8 INT8 TensorRT + ByteTrack)"]
        VisionEngine --> SetpointGen["Kinematic PID Setpoint Generator\n(3D Error Vector -> Velocity Cmds)"]
        SetpointGen --> MAVLinkDaemon["MAVLink Comms Engine\n(cStandard v2.0 / UART DMA)"]
        MAVLinkDaemon --> SBCDiag["Diagnostics & Watchdog Supervisor"]
    end

    subgraph FC ["FLIGHT CONTROLLER (STM32H753 MCU @ 480MHz)"]
        direction TB
        HAL["Hardware Abstraction Layer (HAL / ChibiOS)\n(DMA SPI / I2C / UART Drivers)"] --> Sensors["Core Sensors Suite\n(ICM-42688-P Dual IMUs, DPS310 Baro, M10 GPS)"]
        Sensors --> EKF["EKF3 Navigation Filter\n(24-State Vector Estimator @ 400Hz)"]
        EKF --> Arbitrator["Mode Switch & Safety Arbitrator\n(Offboard vs Manual Override)"]
        Arbitrator --> Mixer["Actuator Controller & PWM Mixer\n(DShot600 Digital ESC Driver)"]
    end

    BEC --> SBC
    BEC --> FC
    Transmitter -- "CRSF Protocol\n(Manual Sticks / AUX Switches)" --> FC
    MAVLinkDaemon -- "MAVLink v2.0 UART2\n(921,600 Baud)" --> Arbitrator
    Mixer --> Motors["4x Brushless Motors / ESCs"]
```

### System Workflow

1. **Image Capture:** 60 FPS MIPI CSI-2 Raw Bayer Stream directly into zero-copy memory.
2. **Target Inference:** INT8 Accelerated YOLOv8 TensorRT engine (< 10 ms execution).
3. **Distance Estimation:** Pin-hole camera model projection & optical expansion calculation.
4. **Kinematic PID Loop:** Translates visual displacement to 3D velocity setpoints at 50 Hz.
5. **MAVLink Serialization:** Encodes MAVLink Packets over UART.
6. **FC Command Ingestion:** Real-time decoding and validation on the STM32.
7. **Safety Arbitration:** Evaluates manual override vs autonomous mode (400 Hz Loop).
8. **Motor Execution:** Translates to DShot600 digital motor commands (8 kHz loop).

### Software Architecture

```mermaid
flowchart TB
    subgraph JetsonOS ["COMPANION COMPUTER FIRMWARE (Jetson Linux OS)"]
        direction TB
        AppLayer["Application Layer: Guidance & Tracking Daemon (C++17 Threaded)\n- Vision Processing Node  |  - Target Tracking Node\n- Kinematic PID Engine     |  - MAVLink Daemon"]
        IPC1["POSIX Shared Memory / Zero-Copy Ring Buffers"]
        HAL_SBC["Hardware Abstraction & Acceleration Layer\n- Jetson Multimedia API (V4L2 ISP)  |  - TensorRT 8.x C++ API\n- Linux TTY Serial DMA Driver"]
        AppLayer --> IPC1 --> HAL_SBC
    end

    JetsonOS -- "Serial MAVLink Stream (921,600 Baud)" --> FCOS

    subgraph FCOS ["FLIGHT CONTROLLER FIRMWARE (ArduPilot / PX4 RTOS)"]
        direction TB
        ModeManager["Flight Application & Mode Controller\n- GUIDED / OFFBOARD Flight Mode Manager\n- Safety & Manual Override Arbitrator"]
        IPC2["Inter-Task IPC / RTOS Queues"]
        NavLayer["Core Navigation & Estimation Layer\n- EKF3 Navigation Filter (24-State)  |  - Rate / Att / Pos PID Loops\n- Motor Mixer"]
        HAL_FC["RTOS & Hardware Abstraction Layer (ChibiOS Kernel)\n- Hardware Interrupts (NVIC)  |  - SPI/I2C/UART DMA Drivers\n- Hardware Watchdog Driver"]
        ModeManager --> IPC2 --> NavLayer --> HAL_FC
    end
```

### Safety & Error Handling

- **Target Loss / Occlusion:** ByteTrack Kalman Filter predicts location based on velocity vector for 500ms before falling back to hover.
- **Process Freeze / Software Crash:** FC MAVLink Heartbeat timeout counter (> 500 ms) revokes autonomous control and reverts to Pilot Manual Angle Mode.
- **Telemetry UART Disconnect:** CRC-16 Checksum failures trigger drop of invalid frames; prolonged failures trigger manual fallback.

### Future Scalability

1. **Multi-Camera Stereo Vision Migration:** Dual MIPI CSI-2 interfaces allow for hardware binocular depth mapping.
2. **LiDAR / Time-of-Flight (ToF) Integration:** Augment optic distance tracking with millimeter-accurate altitude measurements.
3. **Model Context Protocol / AI Swarm Extension:** MAVLink setpoint pipeline supports swarm coordination.
4. **Over-The-Air (OTA) Firmware Updates:** Background firmware flashing via dual-bank Flash and Linux systemd frameworks.
