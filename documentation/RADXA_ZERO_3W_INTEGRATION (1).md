# Radxa ZERO 3W (RK3566, 2GB) FPV Visual Interceptor: Master Technical Integration Document

---

## 1. Executive Summary & System Overview

This technical master specification details the complete hardware interconnections, real-time software pipeline, video output architecture, pilot radio command ingestion, and control workflow for integrating the **Radxa ZERO 3W** (Rockchip RK3566 System-on-Chip, 2 GB LPDDR4 RAM) as an onboard companion computer for an FPV Interceptor Drone running **INAV / Betaflight / ArduPilot**.

### Core Functional Objectives
1. **Video Ingestion & Hardware Preprocessing**: Capture live analog video feed using an onboard MS2109 AV-to-USB capture dongle (`/dev/video0`) with zero-copy V4L2 pipelines.
2. **Onboard AI Target Lock**: Run quantized INT8 YOLO target detection accelerated by the RK3566 1.0 TOPS RKNN NPU, coupled with multi-threaded optical flow / CSRT hybrid tracking on ARM Cortex-A55 cores.
3. **Pilot Radio Command Intercept**: Real-time polling of pilot RC switch state (Channel 6 / AUX2 toggle via ELRS / Crossfire receiver passed through FC MSP over UART2).
4. **Visual Servo Guidance & FC Command**: Command the Flight Controller (FC) using `MSP_SET_RAW_RC` at 50 Hz to autonomously track and follow the locked target when pilot engagement is activated.
5. **Zero-Latency Video & Canvas OSD**: Render AI target lock brackets (`[ TARGET LOCK ]`) directly onto the pilot's FPV Goggles screen via FC AT7456E MSP DisplayPort OSD (Method 2) with 0 ms video transmission delay.
6. **Instant Manual Failsafe**: Instantaneous return of 100% manual stick control to the pilot when the AUX switch is disengaged.

---

## 2. Hardware Specifications & Resource Allocation

| Component | Technical Specification | Operational Allocation |
| :--- | :--- | :--- |
| **SoC** | Rockchip RK3566 (Quad-Core Cortex-A55 @ 1.8 GHz) | Core 0: OS & Telemetry; Core 1-2: OpenCV/CSRT Tracker; Core 3: Control Loop & Serial I/O |
| **NPU** | Rockchip NPU v2 (1.0 TOPS @ INT8) | Accelerated `yolov8n.rknn` inference @ 30–50 FPS |
| **VPU / ISP** | Hardware H.264/H.265 Decoder & RGA 2D Engine | Fast frame resize, color space conversion (YUV420 to RGB/BGR) |
| **RAM** | 2 GB LPDDR4 (Shared Architecture) | ~400 MB OS + 350 MB NPU Buffer + 250 MB OpenCV Memory |
| **Primary I/O** | 40-Pin Header, MIPI-CSI (2-lane), USB 2.0/3.0 Type-C | UART2 (`/dev/ttyS2`) @ 115200 / 460800 baud to FC |
| **Power Consumption**| 5V DC @ 0.6A Idle / 1.2A Peak (~3.0W - 6.0W) | Powered via dedicated 5V/3A BEC from Flight Controller / PDB |

---

## 3. Comprehensive Hardware Connection & Complete Wiring Guide

### 3.1 40-Pin GPIO Header Layout & Pin Map (Radxa ZERO 3W)

```
                            Radxa ZERO 3W 40-Pin Header
                                +-------------------+
    +5V IN (BEC Red 24AWG) ---> |  2 [x]   [x] 1    | NC (3.3V Out - DO NOT FEED 5V)
    +5V IN (BEC Red 24AWG) ---> |  4 [x]   [x] 3    | SDA (I2C)
     GND (BEC Black 24AWG) ---> |  6 [x]   [x] 5    | SCL (I2C)
FC UART RX (Yellow 28AWG) <--- |  8 [x]   [x] 9    | GND (Shield Signal GND)
FC UART TX (White 28AWG)  ---> | 10 [x]   [x] 11   | GPIO
                                | 12 [x]   [x] 13   | GPIO
                                | ...     ...       |
                                +-------------------+
```

---

### 3.2 Master System Wiring Schematic

```
========================================================================================================================
                                     COMPLETE HARDWARE WIRING SCHEMATIC
========================================================================================================================

                 [ 3S - 6S LiPo Battery (11.1V - 25.2V) ]
                                    |
                    +---------------+---------------+
                    | (Main Battery Power Leads)    |
                    v                               v
         +--------------------+           +-------------------+
         | Standalone BEC     |           | Flight Controller |
         | (5V DC / 3A Output)|           | (SpeedyBee F405)  |
         +---------+----------+           +---------+---------+
                   |                                |
         +---------+---------+                      |
         | 5V+ & GND (Power) |                      |
         v                   v                      v
+----------------------------------+       +------------------+
| RADXA ZERO 3W (RK3566, 2GB RAM)  |       | FPV CAMERA       |
|                                  |       | (Caddx / RunCam) |
| Pin 2  : +5V IN (Red Wire)       |       +--------+---------+
| Pin 4  : +5V IN (Red Wire)       |                |
| Pin 6  : GND    (Black Wire)     |                | VIDEO_OUT Wire (Green)
| Pin 8  : UART2_TX (Yellow Wire)  |------------+   | (Splits into 2 lines)
| Pin 10 : UART2_RX (White Wire)   |---------+  |   |
| Pin 9  : GND      (Black Wire)   |-----+   |  |   +-----------------------+
| USB-C  : Host Port               |     |   |  |                           |
+----------------+-----------------+     |   |  |                           | Line B: Video Signal
                 |                       |   |  |                           v
                 | USB Digital Video     |   |  |                +--------------------------+
                 v                       |   |  |                | MS2109 USB Capture Dongle|
+----------------------------------+     |   |  |                | Input: Video + GND       |
| MS2109 USB Capture Dongle        |<----+---+--+----------------| Output: USB-C Plug       |
+----------------------------------+     |   |                   +--------------------------+
                                         |   |
                                         |   | Line A: Direct Video Signal
                                         |   v
                               +---------+------------------+
                               | FLIGHT CONTROLLER PADS     |
                               |                            |
                               | Pad CAM : Video Signal In  |<--- (Line A from Camera)
                               | Pad VTX : Video Signal Out |----+
                               |                            |    |
                               | Pad R2  : UART2 RX         |<---+ (From Radxa Pin 8 TX)
                               | Pad T2  : UART2 TX         |<---+ (From Radxa Pin 10 RX)
                               | Pad GND : Signal Ground    |<---+ (From Radxa Pin 9 GND)
                               |                            |
                               | Pad R1  : UART1 RX         |<---+ (From Receiver TX)
                               | Pad T1  : UART1 TX         |<---+ (From Receiver RX)
                               | Pad 5V  : 5V Power Out     |----+
                               | Pad GND : Ground           |--+ |
                               +----------------------------+  | |
                                                               | |
                               +----------------------------+  | |
                               | RADIO RECEIVER (ELRS)      |  | |
                               | Pad VCC : 5V Power In      |<---+
                               | Pad GND : Ground           |<---+
                               | Pad TX  : CRSF TX          |
                               | Pad RX  : CRSF RX          |
                               +----------------------------+
                                                                 |
                                                                 v
                                                       +--------------------+
                                                       | 5.8GHz FPV VTX     |
                                                       | Pad VIDEO_IN       |
                                                       | (Sends camera feed |
                                                       | + AI OSD to Goggles|
                                                       +--------------------+
```

---

### 3.3 Complete Pin-by-Pin Wiring Matrix

| Source Component | Source Pin / Pad | Wire Color & AWG | Target Component | Target Pin / Pad | Signal & Protocol Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **5V Standalone BEC** | `5V OUT (+)` | Red (24 AWG) | **Radxa ZERO 3W** | Pin 2 (`+5V_IN`) | Primary +5V DC Power Input |
| **5V Standalone BEC** | `5V OUT (+)` | Red (24 AWG) | **Radxa ZERO 3W** | Pin 4 (`+5V_IN`) | Secondary +5V DC Power Input (Parallel line for current capacity) |
| **5V Standalone BEC** | `GND OUT (-)` | Black (24 AWG) | **Radxa ZERO 3W** | Pin 6 (`GND`) | Main DC Power Ground Reference |
| **LiPo Battery / PDB** | `VBAT (+)` | Red (20 AWG) | **5V Standalone BEC** | `VIN (+)` | 7V–26V DC Input to Step-Down BEC |
| **LiPo Battery / PDB** | `GND (-)` | Black (20 AWG) | **5V Standalone BEC** | `VIN (-)` | Battery Ground Reference |
| **Radxa ZERO 3W** | Pin 8 (`UART2_TX`) | Yellow (28 AWG) | **Flight Controller** | Pad `R2` (UART2 RX) | 3.3V TTL Serial Out (`MSP_SET_RAW_RC` @ 50Hz & OSD Canvas) |
| **Radxa ZERO 3W** | Pin 10 (`UART2_RX`)| White (28 AWG) | **Flight Controller** | Pad `T2` (UART2 TX) | 3.3V TTL Serial In (`MSP_RC` / Telemetry) |
| **Radxa ZERO 3W** | Pin 9 (`GND`) | Black (28 AWG) | **Flight Controller** | Pad `GND` | Signal Reference Ground (Eliminates UART noise) |
| **Radio Receiver (ELRS)**| `5V (+)` | Red (28 AWG) | **Flight Controller** | Pad `5V` (FC Internal) | Receiver Power Supply |
| **Radio Receiver (ELRS)**| `GND (-)` | Black (28 AWG) | **Flight Controller** | Pad `GND` | Receiver Ground |
| **Radio Receiver (ELRS)**| `TX` | Green (28 AWG) | **Flight Controller** | Pad `R1` (UART1 RX) | CRSF / SBUS Protocol Serial Stream |
| **Radio Receiver (ELRS)**| `RX` | Blue (28 AWG) | **Flight Controller** | Pad `T1` (UART1 TX) | Telemetry Feedback to Transmitter |
| **FPV Camera (CVBS)** | Video Signal Wire | Green (28 AWG) | **FC Pad `CAM`** | Pad `CAM` | Line A: Direct raw video feed to FC OSD chip |
| **FPV Camera (CVBS)** | Video Signal Wire | Green (28 AWG) | **MS2109 Dongle** | Video IN Pad | Line B: Parallel video feed to USB capture dongle |
| **MS2109 Dongle** | USB-C Male | USB-C OTG | **Radxa ZERO 3W** | USB 2.0 Host Port | Digital UVC Video stream (`/dev/video0`) |

---

### 3.4 Electrical Standards & Wiring Assembly Rules

1. **Power Wire Gauge**:
   * **Battery to 5V BEC**: 20 AWG or 22 AWG silicone wire.
   * **BEC 5V to Radxa Header**: Dual 24 AWG red wires to Pin 2 and Pin 4 to prevent voltage sag during NPU peak currents ($> 1.2\text{ A}$).
2. **Signal Wire Gauge**: 28 AWG or 30 AWG twisted-pair wires for UART connections (`TX`/`RX` wrapped with `GND`).
3. **Common Ground Plane**: All grounds (LiPo GND, BEC GND, Radxa GND, FC GND, Receiver GND, MS2109 GND) **MUST be tied to a single common ground plane** to eliminate ground loop voltage offsets.
4. **Logic Level Compatibility**: Radxa ZERO 3W GPIO pins operate strictly at **3.3V TTL**. FC UART pads operate natively at **3.3V TTL**. No voltage level shifters are required.

---

### 3.5 Software Port Configuration Guide (INAV / Betaflight)

* **Ports Tab**:
  * `UART1`: Enable **Serial RX** (For ELRS/Crossfire receiver @ 420000 baud CRSF protocol).
  * `UART2`: Enable **MSP** @ `115200` baud (or `460800` baud) for Radxa ZERO 3W companion computer link.
* **Receiver Tab**:
  * Receiver Type: `Serial-based receiver`
  * Serial Receiver Provider: `CRSF`
  * Channel Map: `AETR1234`
* **Modes Tab**:
  * Mode `ARM`: `AUX1` (Channel 5) $\rightarrow 1800 - 2100\text{ }\mu s$
  * Mode `ANGLE`: `AUX2` (Channel 6) $\rightarrow 1800 - 2100\text{ }\mu s$ (Engages interceptor follow when pilot flips switch HIGH).

---

## 4. Video Output Architecture & Method Selection

### 4.1 Comparison of All 4 Video Output Methods

```
                            +---------------------------------------+
                            |     RADXA ZERO 3W (RK3566 SBC)        |
                            +---+-----------+-----------+-------+---+
                                |           |           |       |
      +-------------------------+           |           |       +-------------------------+
      | Method 1                            | Method 2  | Method 3                        | Method 4
      v                                     v           v                                 v
+------------------+              +------------------+ +------------------+     +------------------+
| Micro HDMI Port  |              | UART2 MSP OSD    | | USB-to-AV Dongle |     | Wi-Fi H.264 RTSP |
| (1080p @ 60Hz)   |              | (Canvas Overlay) | | (CVBS Analog Out)|     | (UDP Wireless)   |
+--------+---------+              +--------+---------+ +--------+---------+     +--------+---------+
         |                                 |                  |                          |
         v                                 v                  v                          v
+------------------+              +------------------+ +------------------+     +------------------+
| HD FPV Goggles / |              | INAV OSD Chip    | | 5.8GHz FPV VTX   |     | Ground Station   |
| Field Monitor    |              | -> Analog VTX    | | (Analog Video)   |     | Laptop / Tablet  |
+------------------+              +------------------+ +------------------+     +------------------+
```

1. **Method 1 (Hardware Micro HDMI)**: Direct 1080p@60Hz video via Micro HDMI port to external HD monitors or HD FPV goggles.
2. **Method 2 (FC MSP Canvas OSD - SELECTED)**: Raw camera feed passes directly to FC OSD chip for zero camera latency. Radxa sends AI target coordinates via UART2 MSP DisplayPort Canvas packets. FC draws `[ TARGET LOCK ]` brackets directly on pilot FPV goggles.
3. **Method 3 (USB-to-CVBS Analog Dongle)**: Radxa outputs composited video with drawn boxes back through an analog USB DAC output to VTX.
4. **Method 4 (Wi-Fi H.264 RTSP Stream)**: Wireless streaming over 5GHz Wi-Fi to ground station laptops/tablets using RK3566 hardware VPU.

---

### 4.2 Hardware Selection for Method 2

1. **Analog FPV Camera**: Caddx Ratel 2 / RunCam Phoenix 2 / Foxeer Toothless 2 (NTSC/PAL 1200TVL).
2. **Analog CVBS to USB UVC Video Grabber**: **MS2109 / MS2130 USB AV-to-UVC Board** (~3.5g stripped decased PCB).
3. **Flight Controller (FC)**: SpeedyBee F405 V3/V4, Matek H743, or Kakute H7 with onboard **AT7456E analog OSD chip**.
4. **5.8GHz FPV VTX**: TBS Unify Pro32, Rush Tank Solo, or SpeedyBee TX800.

---

## 5. Pilot Command Ingestion, AI Processing & FC Steering Loop

### 5.1 End-to-End Command & Data Loop Schematic

```
+-------------------------------------------------------------------------------------------------------+
|                                    1. PILOT COMMAND INGESTION                                         |
|                                                                                                       |
|  [ Pilot Transmitter ] ---- (2.4GHz RF) ----> [ Receiver ] ---- (CRSF UART1) ----> [ INAV FC ]      |
|                                                                                               |       |
|  [ Radxa ZERO 3W ] <---- (MSP_RC UART2 @ 50Hz) -----------------------------------------------+       |
|   Polls Channel 6 (AUX2 Engage Switch Value)                                                          |
+-------------------------------------------------------------------------------------------------------+
                                                   |
                                                   v
+-------------------------------------------------------------------------------------------------------+
|                                    2. ONBOARD RADXA PROCESSING                                        |
|                                                                                                       |
|   IF AUX2 > 1700 us (ENGAGED) & Target Locked:                                                        |
|     * Capture Camera Frame (/dev/video0 via MS2109 USB Capture)                                       |
|     * Run RK3566 NPU YOLOv8 Model -> Compute Target Bounding Box (cx, cy)                             |
|     * Compute Pixel Offset:  dX = cx - Frame_Width/2,  dY = cy - Frame_Height/2                        |
|     * Run PID Aim Controller:                                                                         |
|         Roll_Value  = 1500 + PID_roll(dX)  // Coordinated bank-to-turn                               |
|         Pitch_Value = 1500 + PID_pitch(dY)                                                           |
|         Yaw_Value   = 1500 + PID_yaw(dX)                                                             |
+-------------------------------------------------------------------------------------------------------+
                                                   |
                                                   v
+-------------------------------------------------------------------------------------------------------+
|                                    3. COMMAND OUTPUT TO FC                                            |
|                                                                                                       |
|   Radxa constructs MSP_SET_RAW_RC (Cmd 200) binary packet:                                            |
|   Payload: [Roll=Roll_Value, Pitch=Pitch_Val, Throttle=1500, Yaw=Yaw_Val, AUX1=1800, AUX2=1900, ...] |
|                                                                                                       |
|   [ Radxa ZERO 3W ] ---- (MSP_SET_RAW_RC over UART2 Pin 8 -> FC Pad R2 @ 50Hz) ----> [ INAV FC ]     |
|   INAV FC overrides stick inputs & drives ESCs/Motors to track target                                 |
+-------------------------------------------------------------------------------------------------------+
```

---

### 5.2 Step-by-Step Execution Logic

1. **Pilot Command Ingestion (FC $\rightarrow$ Radxa)**:
   * Every $20\text{ ms}$ ($50\text{ Hz}$), Radxa sends an `MSP_RC` request packet (Command 105) over UART2 to FC.
   * FC responds with current RC channel values ($\mu s$). Radxa inspects Channel 6 (`AUX2`):
     * **AUX2 < 1300 $\mu s$ (OFF)**: Standby mode. Radxa sends NO override commands. Pilot has 100% manual stick control.
     * **AUX2 > 1700 $\mu s$ (ENGAGED)**: Radxa engages autonomous visual tracking and steering.

2. **AI Vision & PID Steering Calculation (Radxa NPU)**:
   * Radxa grabs camera frame from `/dev/video0` (MS2109 USB capture).
   * RK3566 NPU runs `yolov8n.rknn` (INT8) and calculates target bounding box centroid $(cx, cy)$.
   * Computes optical center error: $\Delta X = cx - 640$, $\Delta Y = cy - 360$.
   * FPV Follow PID controller calculates dynamic stick command values ($\mu s$ range 1000–2000), driving a coordinated bank-to-turn intercept:
     * **Roll (Bank):** Maps to target X-axis error ($\Delta X$) to lean the drone into the turn, preventing side-slip and maintaining smoother high-speed cornering.
     * **Pitch (Elevation):** Maps to target Y-axis error ($\Delta Y$) to ascend or descend toward the target, keeping it centered vertically.
     * **Yaw (Heading):** Maps to target X-axis error ($\Delta X$) to pan the nose horizontally toward the target.
     $$\text{Roll}_{\text{channel}} = 1500 + \left( K_{p\_roll} \cdot \Delta X + K_{d\_roll} \cdot \frac{d\Delta X}{dt} \right)$$
     $$\text{Pitch}_{\text{channel}} = 1500 - \left( K_{p\_pitch} \cdot \Delta Y + K_{d\_pitch} \cdot \frac{d\Delta Y}{dt} \right)$$
     $$\text{Yaw}_{\text{channel}} = 1500 + \left( K_{p\_yaw} \cdot \Delta X + K_{d\_yaw} \cdot \frac{d\Delta X}{dt} \right)$$

3. **Command Output Back to FC (`MSP_SET_RAW_RC` @ 50 Hz)**:
   * Radxa constructs a 38-byte binary **MSP_SET_RAW_RC** (Command 200) packet containing 16 channel values.
   * Streams packet over UART2 Pin 8 (`UART2_TX`) to FC Pad `R2` at **50 Hz**.
   * INAV FC overrides physical transmitter sticks and drives ESC motors to steer drone toward target in ANGLE flight mode.

4. **Instant Manual Failsafe**:
   * When pilot switches `AUX2` to LOW ($< 1300\ \mu s$), Radxa immediately halts sending `MSP_SET_RAW_RC`.
   * INAV FC MSP override safety timer ($200\text{ ms}$) expires. INAV FC automatically drops MSP override and restores 100% manual stick control to pilot transmitter.

---

## 6. System State Machine & Execution Workflow

```
                       +-----------------------+
                       |     STATE 0: IDLE     |
                       | System Initialization |
                       +-----------+-----------+
                                   |
                                   v
                       +-----------------------+
                       |    STATE 1: SEARCH    |
                       | Camera Capture Active |
                       |  NPU Scanning Target  |
                       +-----------+-----------+
                                   |
                         Target Detected & Locked
                                   |
                                   v
                       +-----------------------+
                       |    STATE 2: LOCKED    |
                       | Hybrid Tracker Active |
                       | Standby for Pilot CMD |
                       +-----------+-----------+
                                   |
                       Pilot Flips AUX2 Switch HIGH
                                   |
                                   v
                       +-----------------------+
        +------------->|   STATE 3: ENGAGED    |
        |              | Visual Servo Control  |
        |              |  MSP_SET_RAW_RC @50Hz |
        +--------------+-----------+-----------+
                                   |
                    +--------------+--------------+
                    |                             |
         Target Lost > Max Frames       Pilot Flips AUX2 LOW
                    |                             |
                    v                             v
        +-----------------------+     +-----------------------+
        |   STATE 4: LOST/REACQ |     |   STATE 1: SEARCH     |
        | CSRT / Search Pattern |     | Revert Manual Flight  |
        +-----------------------+     +-----------------------+
```

---

## 7. Power Supply & Thermal Management Guidelines

1. **Power Budget**: Peak CPU + NPU draw is **5.2W** (5.0V @ 1.04A). Power Radxa ZERO 3W via dedicated **5V / 3A BEC** connected to LiPo battery leads (do NOT use FC internal 500mA BEC).
2. **Thermal Cooling**: Install low-profile aluminum heatsink over RK3566 SoC. FPV propwash airflow inside drone frame maintains SBC temperature under **55°C** under continuous load.

---

## 8. Eliminating & Replacing the MS2109 USB Capture Dongle

### 8.1 Technical Principle: Why Pure Software Cannot Decode Analog CVBS directly
An analog FPV camera outputs a composite high-frequency continuous electrical AC waveform (**NTSC/PAL CVBS** signal at 5.5 MHz). Because digital processors like the Rockchip RK3566 operate strictly on binary digital data ($0$s and $1$s), **software code alone cannot convert raw analog voltage into pixels without hardware Analog-to-Digital Converter (ADC) silicon**.

However, you can **completely replace or eliminate the MS2109 USB Dongle hardware** using much cleaner, lighter, and lower-latency hardware architectures!

---

### 8.2 Solution 1: Direct MIPI-CSI Digital Camera (RECOMMENDED ALTERNATIVE)

Instead of converting analog video to USB, connect a native **MIPI-CSI Digital Camera Module** directly to the Radxa ZERO 3W **22-pin 2-lane MIPI CSI FPC Connector**.

```
+------------------------------------+                  +-----------------------------------+
| MIPI-CSI Digital Camera            |                  | RADXA ZERO 3W (RK3566, 2GB)       |
| (IMX219 / OV5647 / OV9281)         |== (22-pin FPC) ==| Native MIPI CSI Input Connector   |
| 1080p @ 60 FPS / 720p @ 120 FPS    |   Ribbon Cable   | Direct Hardware DMA Buffer        |
+------------------------------------+                  +-----------------------------------+
```

#### Hardware Component Recommendations for Solution 1:
* **Camera Module**: Raspberry Pi Camera v2 (IMX219 8MP), OV5647 5MP sensor, or **OmniVision OV9281 Global Shutter** camera (for ultra-high-speed motion blur elimination during high-g intercepts).
* **Connection**: 22-pin to 15-pin FPC flexible ribbon cable plugged directly into Radxa ZERO 3W CSI socket.

#### Key Advantages over MS2109 USB Dongle:
1. **Zero USB Dongle Required**: Eliminates external USB capture hardware, cables, and connectors completely.
2. **Ultra-Low Latency**: **~3–5 ms capture latency** (compared to ~35–50 ms latency through MS2109 USB conversion).
3. **Zero CPU Overhead**: Hardware DMA zero-copy buffer transfers video frames directly into RK3566 NPU memory (`/dev/video0`).
4. **Weight Savings**: Total payload weight is **< 3 grams** (Micro camera + ribbon cable).
5. **Higher Frame Rate**: Native support for **60 FPS to 120 FPS** input streams for high-speed tracking.

---

### 8.3 Solution 2: Dual-Camera Architecture (Dedicated FPV + Direct MIPI AI Camera)

If you want to maintain a standard analog FPV camera for the human pilot while giving the Radxa ZERO 3W ultra-low latency AI vision:

```
[ Pilot Analog Camera (Caddx) ] -----> [ INAV FC OSD ] -----> [ 5.8GHz VTX ] -----> [ Pilot FPV Goggles ]
  (Dedicated Human Pilot View)

[ MIPI-CSI AI Camera (OV5647) ] -----> [ Radxa ZERO 3W (CSI Connector) ] -----> [ RK3566 NPU AI Engine ]
  (Dedicated Onboard AI View)
```

1. **Camera 1 (Pilot FPV)**: Ultra-lightweight 1.5g Analog Nano Camera connected directly to FC `CAM` pad for the pilot's FPV Goggles feed (**0 ms video delay**).
2. **Camera 2 (AI Target Lock)**: Micro MIPI-CSI Digital Camera plugged into Radxa ZERO 3W MIPI port for target detection.
3. **Control Output**: Radxa computes target error and commands INAV FC over UART2 via `MSP_SET_RAW_RC` @ 50 Hz.

---

### 8.4 Solution 3: Onboard CVBS-to-MIPI Decoder IC (Replacing Dongle Enclosure)

If you must use an analog camera signal but want to eliminate the bulky USB dongle box:
* **Hardware**: Use a micro **Analog CVBS-to-MIPI CSI bridge module** based on the **ADV7180** or **TVP5150** IC.
* **Size**: Micro PCB ($15\text{ mm} \times 15\text{ mm}$, weight $< 1.5\text{ g}$).
* **Connection**: Analog camera video wire $\rightarrow$ ADV7180 Module $\rightarrow$ FPC Ribbon Cable $\rightarrow$ Radxa ZERO 3W MIPI CSI connector.

---

### 8.5 Master Performance Comparison Matrix

| Parameter | MS2109 USB Dongle | Solution 1: Direct MIPI-CSI Camera | Solution 2: Dual Camera | Solution 3: CVBS-to-MIPI IC |
| :--- | :--- | :--- | :--- | :--- |
| **Extra USB Hardware?** | **YES (USB Dongle)** | **NO (Zero Dongle)** | **NO (Zero Dongle)** | **NO (Micro IC Chip)** |
| **Video Capture Latency**| ~35 – 50 ms | **~3 – 5 ms (Ultra-Fast)** | **~3 – 5 ms (Ultra-Fast)** | ~15 – 20 ms |
| **Frame Rate Capacity** | 30 – 50 FPS | **60 – 120 FPS** | **60 – 120 FPS** | 30 – 60 FPS |
| **System Payload Weight**| + 12g (or ~3.5g bare) | **< 3 grams total** | ~5 grams total | ~2 grams total |
| **CPU DMA Overhead** | High USB IRQ load | **0% CPU (Hardware DMA)** | **0% CPU (Hardware DMA)** | **0% CPU (Hardware DMA)** |
| **Mechanical Reliability**| Medium (USB connector) | **High (Locked FPC Clip)**| **High (Locked FPC Clip)**| **High (Direct Soldered)** |

