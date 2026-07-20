# FPV Interceptor AI — Visual Lock & Follow Guide

PC vision lock for an **INAV FPV drone**: camera tracks a target, then MSP `SET_RAW_RC` steers **yaw + pitch** to keep it centered (ANGLE mode recommended).

> **Safety:** Bench with **props off** first. This sends real RC channel values to the flight controller.

---

## 1. What it does

| Stage | Behavior |
|--------|----------|
| **Lock** | Mouse-drag ROI (`L`) or YOLO auto-lock (`Y`) |
| **Track** | CSRT between frames + YOLO refresh / reacquire |
| **Follow** | FPV visual servo → yaw & pitch sticks toward target center |
| **FC** | Continuous MSP AETR + ARM (CH5) + flight mode (CH6) |

```
Camera → YOLO/CSRT lock → pixel error → FPVFollowController
                                              ↓
                                    MSP_SET_RAW_RC @ 50 Hz
                                              ↓
                                         INAV FC (ANGLE)
```

---

## 2. Project layout (cleaned)

```
inercepterAI/
├── main.py                      # Entry point — MSP lock + follow
├── calibration_fpv.py           # GUI calibrator (save → calibration.json)
├── calibration.json             # Saved camera / follow values
├── config.py                    # YOLO / tracker defaults
├── requirements.txt
├── yolov8n.pt                   # Default COCO weights (auto-used)
├── control/
│   └── fpv_follow.py            # Yaw/pitch aim PID + lead
├── detection/
│   ├── yolo_detector.py         # Ultralytics YOLO wrapper
│   └── hybrid_tracker.py        # YOLO + OpenCV CSRT/KCF hybrid
├── utils/
│   ├── calib_io.py              # Load / save calibration.json
│   ├── helpers.py
│   └── logger.py
├── models/
├── datasets/drone_missile/
└── scripts/train_drone_missile.py
```

---

## 3. Hardware & INAV setup

### Required

| Item | Notes |
|------|--------|
| FPV / USB camera | Seen by Windows as a camera index |
| Flight controller | INAV with MSP on a UART |
| USB–serial link | e.g. FTDI / onboard USB VCP → `COMx` |
| Channel map | **AETR** (default in `main.py`) |

### INAV Modes tab (typical)

| Mode | Channel | Range |
|------|---------|--------|
| **ARM** | CH5 / AUX1 | high ≈ 1800 |
| **ANGLE** (or HORIZON) | CH6 / AUX2 | high ≈ 1900 |

MSP receiver / override must accept `MSP_SET_RAW_RC` from the PC link you use.

### Wire check

1. Connect FC USB (or UART↔USB) → note COM port in Device Manager  
2. Set `CONTROL_PORT` in `main.py` (default `"COM4"`)  
3. Set `CONTROL_BAUD` to match the MSP UART (often `57600` or `115200`)  
4. Set `CAMERA_INDEX` (`0`, `1`, …) until the preview is correct  

---

## 4. Install

```bash
cd inercepterAI
python -m venv .venv

# Windows
.venv\Scripts\activate

pip install -r requirements.txt
```

### Optional: CUDA (much faster YOLO)

```bash
pip uninstall -y torch torchvision
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
python -c "import torch; print(torch.cuda.is_available())"
```

CPU works; use `YOLO_EVERY_N = 4`–`6` in `main.py` if FPS drops.

---

## 5. Run

### Calibrate first (recommended)

```bash
python calibration_fpv.py
```

**Props off.** Workflow:

1. **Connect FC** → set COM port → **ANGLE ON**  
2. **Hold** Yaw LEFT/RIGHT and Pitch UP/DOWN — craft should move that way (meters + FC)  
3. **Open camera** → drag-lock a target  
4. Enable **LIVE FOLLOW** — craft should aim at the box  
5. If it turns the **wrong** way → **Flip Yaw** / **Flip Pitch**  
6. Tune Kp / max with **+ / −**  
7. **Save JSON** → `calibration.json` (auto-loaded by `main.py`)

Hold-to-move buttons send real MSP sticks. Live follow sends the same yaw/pitch the flight code will use.

### Fly / bench follow

```bash
python main.py
```

---

## 6. Controls

| Key | Action |
|-----|--------|
| **L** | Start lock selection — drag a box on the target, release |
| **Y** | YOLO auto-lock highest-confidence detection |
| **E** | Enable follow assist |
| **D** | Disable follow assist |
| **A** | Arm CH5 + flight mode CH6 ON |
| **X** | Disarm + mode OFF, throttle → 1000 |
| **M** | Toggle flight mode CH6 only |
| **0** | Force CH6 = 1900 (mode ON) |
| **U** / **J** | Throttle +25 / −25 |
| **R** | Clear lock / tracker |
| **S** | Query FC arm status (MSP_STATUS) |
| **Q** | Quit (disarms safely) |

### Bench procedure (props off)

1. `python main.py`  
2. Confirm HUD shows RC values updating  
3. Press **0** or **M** → CH6 high (ANGLE)  
4. **L** → drag box on subject (or **Y** if YOLO sees it)  
5. Assist usually enables on lock — watch ERR X/Y and yaw/pitch move toward center  
6. **A** only when you intentionally test armed behavior (props off)  
7. **X** then **Q** to exit  

---

## 7. How following works (FPV)

File: `control/fpv_follow.py`

| Axis | Image error | Stick |
|------|-------------|--------|
| **Yaw** | Target left/right of center | Turn to face target |
| **Pitch** | Target above/below center | Nose toward target |
| **Roll** | Off by default | Optional light strafe (`use_roll=True`) |

Extras for accuracy:

- Normalized error (works at any resolution)  
- Soft deadzone + expo (precise near center, stronger when far)  
- Lead from image-plane velocity  
- Filtered D-term + slew limits (smooth sticks)  

Tune in `main.py` → `FPV_CFG`:

```python
FPV_CFG = FPVFollowConfig(
    yaw_kp=340.0,
    pitch_kp=310.0,
    max_yaw=400.0,
    max_pitch=360.0,
    lead_s=0.14,
    yaw_dir=1.0,      # flip to -1.0 if left/right inverted
    pitch_dir=-1.0,   # flip to 1.0 if up/down inverted
    use_roll=False,
)
```

| Symptom | Try |
|---------|-----|
| Turns the wrong way | Flip `yaw_dir` |
| Pitches the wrong way | Flip `pitch_dir` |
| Slow to catch target | Raise `yaw_kp` / `pitch_kp` or `max_yaw` / `max_pitch` |
| Oscillates around center | Lower Kp, raise `deadzone_norm`, or lower `lead_s` |
| Jerky sticks | Lower `out_alpha`, raise slew values slightly |

---

## 8. How tracking works

File: `detection/hybrid_tracker.py`

1. Lock from ROI or YOLO box  
2. **CSRT** (or KCF) tracks every frame  
3. Every `YOLO_EVERY_N` frames, YOLO re-detects and snaps back by IoU / nearest center  
4. If OpenCV loses the target, YOLO reacquire runs until `MAX_LOST_FRAMES`

In `main.py`:

```python
USE_YOLO = True
YOLO_EVERY_N = 4
TRACKER_TYPE = "CSRT"   # or "KCF" for more speed
```

Detection defaults live in `config.py` (`DetectionConfig`):

| Mode | When to use |
|------|-------------|
| `"coco"` (default) | Works out of the box with `yolov8n.pt` |
| `"world"` | Open-vocab prompts (`drone`, `UAV`, …) — needs YOLO-World weights |
| `"custom"` | After training `models/drone_missile_best.pt` |

---

## 9. Optional: train a drone detector

1. Label images (Ultralytics layout) under `datasets/drone_missile/`  
2. Copy `data.yaml.example` → `data.yaml` and set class names  
3. Train:

```bash
python scripts/train_drone_missile.py
```

4. Set in `config.py`:

```python
mode: DetectionMode = "custom"
```

---

## 10. Key settings cheatsheet (`main.py`)

| Setting | Meaning |
|---------|---------|
| `CONTROL_PORT` | FC serial port |
| `CONTROL_BAUD` | MSP baud |
| `CAMERA_INDEX` | OpenCV camera index |
| `FRAME_WIDTH` / `HEIGHT` | Capture size |
| `ARM_CH` / `MODE_CH` | 0-based indices (CH5=4, CH6=5) |
| `MODE_ON_VALUE` | 1900 for ANGLE range |
| `SEND_HZ` | MSP send rate (default 50) |
| `USE_YOLO` | Hybrid refresh on/off |
| `FPV_CFG` | Follow gains / directions |

---

## 11. Troubleshooting

| Problem | Fix |
|---------|-----|
| `Could not open COMx` | Wrong port; unplug other serial tools; check baud |
| Camera black / wrong feed | Change `CAMERA_INDEX`; close other apps using the cam |
| YOLO load fails | `pip install ultralytics torch`; keep `yolov8n.pt` in project root |
| Low FPS | Raise `YOLO_EVERY_N`, use `TRACKER_TYPE = "KCF"`, install CUDA torch |
| Tracks but does not follow | Press **E**; confirm ASSIST ON; check ANGLE on CH6 |
| Follows opposite direction | Flip `yaw_dir` / `pitch_dir` in `FPV_CFG` |
| FC ignores MSP | Enable MSP on that UART; check INAV “Receiver” / MSP override |
| Arm fails | Throttle low, ANGLE on, arm switch range matches INAV Modes |

---

## 12. Safety checklist

- [ ] Props off for first software tests  
- [ ] ARM / ANGLE ranges verified in INAV Modes  
- [ ] Know **X** (disarm) and **Q** (quit)  
- [ ] Start with assist on the bench; only then consider props-on outdoor tests with a buddy box / kill switch  
- [ ] Treat AI follow as an experiment — always keep a manual override path  

---

## Quick start

```bash
.venv\Scripts\activate
pip install -r requirements.txt
# edit CONTROL_PORT / CAMERA_INDEX in main.py
python main.py
```

**L** → drag lock → watch error shrink with assist → **X** / **Q** when done.
