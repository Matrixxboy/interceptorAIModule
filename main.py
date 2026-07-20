"""
FPV MSP visual lock + follow (INAV AETR).

- Non-blocking mouse-drag lock (L + drag)
- Optional YOLO hybrid refresh for better hold / reacquire (Y = auto-lock)
- CSRT local tracker between YOLO frames
- FPV follow: yaw + pitch aim with lead prediction (roll off by default)

Controls:
    L = start manual lock selection mode
    Mouse drag = draw ROI box and release to lock
    Y = YOLO auto-lock best detection
    E = enable visual assist (follow)
    D = disable visual assist
    A = arm CH5 high + flight mode CH6 high
    X = disarm CH5 low + flight mode CH6 low
    M = toggle flight mode only (CH6)
    U = throttle +25
    J = throttle -25
    0 = set flight mode CH6 to 1900
    R = reset tracker
    S = check arm status
    Q = quit

PROPS OFF for bench. Use ANGLE mode on CH6 for FPV follow.
"""

import os
import sys
import cv2
import time
import struct
import serial
from serial.tools import list_ports

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from control.fpv_follow import FPVFollowConfig, FPVFollowController
from utils.calib_io import DEFAULT_PATH, fpv_config_from_dict, load_calibration


# ============================================================
# TELEMETRY / CAMERA CONFIG  (overridden by calibration.json)
# ============================================================

CONTROL_PORT = "COM4"
CONTROL_BAUD = 115200

CAMERA_INDEX = 1
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720


# ============================================================
# INAV MSP SETTINGS
# ============================================================

MSP_SET_RAW_RC = 200
MSP_STATUS = 101

NUM_CHANNELS = 16

RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000

# INAV Channel Map = AETR
ROLL_CH = 0
PITCH_CH = 1
THROTTLE_CH = 2
YAW_CH = 3
ARM_CH = 4          # CH5 / AUX1 — Arm
MODE_CH = 5         # CH6 / AUX2 — Flight mode (ANGLE)

ARM_VALUE = 1800
DISARM_VALUE = 1000

# Flight mode switch (map ANGLE / HORIZON to CH6 in INAV Modes tab)
MODE_ON_VALUE = 1900
MODE_OFF_VALUE = 1000
# When True: arm also enables flight mode, disarm also disables it
MODE_FOLLOWS_ARM = True


# ============================================================
# FPV FOLLOW / TRACKER  (overridden by calibration.json)
# ============================================================

USE_YOLO = True
YOLO_EVERY_N = 4
TRACKER_TYPE = "CSRT"
MIN_BBOX_AREA = 250
MAX_LOST_FRAMES = 60

SEND_HZ = 50

FPV_CFG = FPVFollowConfig(
    deadzone_norm=0.018,
    yaw_kp=340.0,
    yaw_ki=45.0,
    yaw_kd=60.0,
    pitch_kp=310.0,
    pitch_ki=40.0,
    pitch_kd=55.0,
    max_yaw=400.0,
    max_pitch=360.0,
    lead_s=0.14,
    meas_alpha=0.50,
    out_alpha=0.60,
    slew_yaw=1600.0,
    slew_pitch=1400.0,
    yaw_dir=1.0,
    pitch_dir=-1.0,
    use_roll=False,
    rc_mid=RC_MID,
    rc_min=RC_MIN,
    rc_max=RC_MAX,
)


def apply_calibration(path=None) -> None:
    """Load calibration.json and override runtime settings."""
    global CONTROL_PORT, CONTROL_BAUD, CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT
    global USE_YOLO, YOLO_EVERY_N, TRACKER_TYPE, MIN_BBOX_AREA, MAX_LOST_FRAMES
    global SEND_HZ, MODE_ON_VALUE, FPV_CFG, DEADZONE_X, DEADZONE_Y

    calib = load_calibration(path)
    CONTROL_PORT = str(calib.get("control_port", CONTROL_PORT))
    CONTROL_BAUD = int(calib.get("control_baud", CONTROL_BAUD))
    CAMERA_INDEX = int(calib.get("camera_index", CAMERA_INDEX))
    FRAME_WIDTH = int(calib.get("frame_width", FRAME_WIDTH))
    FRAME_HEIGHT = int(calib.get("frame_height", FRAME_HEIGHT))
    USE_YOLO = bool(calib.get("use_yolo", USE_YOLO))
    YOLO_EVERY_N = int(calib.get("yolo_every_n", YOLO_EVERY_N))
    TRACKER_TYPE = str(calib.get("tracker_type", TRACKER_TYPE)).upper()
    MIN_BBOX_AREA = int(calib.get("min_bbox_area", MIN_BBOX_AREA))
    MAX_LOST_FRAMES = int(calib.get("max_lost_frames", MAX_LOST_FRAMES))
    SEND_HZ = int(calib.get("send_hz", SEND_HZ))
    MODE_ON_VALUE = int(calib.get("mode_on_value", MODE_ON_VALUE))
    FPV_CFG = fpv_config_from_dict(calib.get("fpv", {}))
    DEADZONE_X = int(FRAME_WIDTH * 0.5 * FPV_CFG.deadzone_norm)
    DEADZONE_Y = int(FRAME_HEIGHT * 0.5 * FPV_CFG.deadzone_norm)
    print(f"[CALIB] Loaded {path or DEFAULT_PATH}")
    print(
        f"[CALIB] cam={CAMERA_INDEX} port={CONTROL_PORT} tracker={TRACKER_TYPE} "
        f"yaw_dir={FPV_CFG.yaw_dir} pitch_dir={FPV_CFG.pitch_dir}"
    )


# Soft deadzone box drawn on HUD (pixels) — visual only
DEADZONE_X = int(FRAME_WIDTH * 0.5 * FPV_CFG.deadzone_norm)
DEADZONE_Y = int(FRAME_HEIGHT * 0.5 * FPV_CFG.deadzone_norm)

# Apply saved calibration if present
apply_calibration()


# ============================================================
# MOUSE SELECTION STATE
# ============================================================

select_mode = False
tracking_drag = False
drag_start = None
drag_end = None
pending_roi = None


# ============================================================
# BASIC UTILS
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def print_ports():
    print("\nAvailable serial ports:")
    for p in list_ports.comports():
        print(f"{p.device} | {p.description} | {p.hwid}")
    print()


def create_tracker():
    t = TRACKER_TYPE.upper()

    if t == "CSRT":
        if hasattr(cv2, "TrackerCSRT_create"):
            return cv2.TrackerCSRT_create()
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
            return cv2.legacy.TrackerCSRT_create()

    if t == "KCF":
        if hasattr(cv2, "TrackerKCF_create"):
            return cv2.TrackerKCF_create()
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerKCF_create"):
            return cv2.legacy.TrackerKCF_create()

    if t == "MOSSE":
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerMOSSE_create"):
            return cv2.legacy.TrackerMOSSE_create()

    raise RuntimeError(
        "Tracker unavailable. Install:\n"
        "pip install opencv-contrib-python"
    )


def normalize_roi(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    x = min(x1, x2)
    y = min(y1, y2)
    w = abs(x2 - x1)
    h = abs(y2 - y1)

    return int(x), int(y), int(w), int(h)


def bbox_center(bbox):
    x, y, w, h = bbox
    return int(x + w / 2), int(y + h / 2)


def bbox_area(bbox):
    x, y, w, h = bbox
    return int(w * h)


def draw_crosshair(frame):
    h, w = frame.shape[:2]
    cx = w // 2
    cy = h // 2

    cv2.drawMarker(
        frame,
        (cx, cy),
        (255, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=35,
        thickness=2,
    )

    cv2.rectangle(
        frame,
        (cx - DEADZONE_X, cy - DEADZONE_Y),
        (cx + DEADZONE_X, cy + DEADZONE_Y),
        (255, 255, 0),
        1,
    )


def mouse_callback(event, x, y, flags, param):
    global select_mode, tracking_drag, drag_start, drag_end, pending_roi

    if not select_mode:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        tracking_drag = True
        drag_start = (x, y)
        drag_end = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE and tracking_drag:
        drag_end = (x, y)

    elif event == cv2.EVENT_LBUTTONUP and tracking_drag:
        tracking_drag = False
        drag_end = (x, y)

        roi = normalize_roi(drag_start, drag_end)

        if roi[2] > 10 and roi[3] > 10:
            pending_roi = roi
        else:
            print("[LOCK] ROI too small. Try again.")

        drag_start = None
        drag_end = None
        select_mode = False


# ============================================================
# MSP FUNCTIONS
# ============================================================

def build_msp_set_raw_rc(channels):
    payload = struct.pack("<" + "H" * len(channels), *channels)
    size = len(payload)

    checksum = MSP_SET_RAW_RC ^ size
    for byte in payload:
        checksum ^= byte
    checksum &= 0xFF

    return b"$M<" + bytes([size, MSP_SET_RAW_RC]) + payload + bytes([checksum])


def build_msp_request(code):
    checksum = code ^ 0
    return b"$M<" + bytes([0, code, checksum])


def read_msp_response(link_serial, timeout=0.5):
    old_timeout = link_serial.timeout
    link_serial.timeout = timeout

    try:
        start = time.time()
        state = 0

        while time.time() - start < timeout:
            b = link_serial.read(1)
            if not b:
                continue

            if state == 0:
                if b == b"$":
                    state = 1
            elif state == 1:
                if b == b"M":
                    state = 2
                else:
                    state = 0
            elif state == 2:
                if b == b">":
                    break
                else:
                    state = 0
        else:
            return None

        size_b = link_serial.read(1)
        code_b = link_serial.read(1)

        if not size_b or not code_b:
            return None

        size = size_b[0]
        code = code_b[0]

        payload = link_serial.read(size)
        checksum = link_serial.read(1)

        if len(payload) != size or not checksum:
            return None

        return {"code": code, "payload": payload}

    finally:
        link_serial.timeout = old_timeout


def send_rc_channels(link_serial, channels, repeat=1, interval=0.0):
    if len(channels) != NUM_CHANNELS:
        raise ValueError(f"Expected {NUM_CHANNELS} channels, got {len(channels)}")

    safe_channels = [int(clamp(ch, RC_MIN, RC_MAX)) for ch in channels]
    packet = build_msp_set_raw_rc(safe_channels)

    for _ in range(repeat):
        link_serial.write(packet)
        if interval > 0:
            time.sleep(interval)


def make_channels(
    roll=1500,
    pitch=1500,
    yaw=1500,
    throttle=1000,
    arm_value=DISARM_VALUE,
    mode_value=MODE_OFF_VALUE,
):
    channels = [1500] * NUM_CHANNELS

    channels[ROLL_CH] = int(clamp(roll, RC_MIN, RC_MAX))
    channels[PITCH_CH] = int(clamp(pitch, RC_MIN, RC_MAX))
    channels[THROTTLE_CH] = int(clamp(throttle, RC_MIN, RC_MAX))
    channels[YAW_CH] = int(clamp(yaw, RC_MIN, RC_MAX))
    channels[ARM_CH] = int(clamp(arm_value, RC_MIN, RC_MAX))
    channels[MODE_CH] = int(clamp(mode_value, RC_MIN, RC_MAX))

    return channels


def send_neutral_disarmed(link_serial):
    ch = make_channels(1500, 1500, 1500, 1000, DISARM_VALUE, MODE_OFF_VALUE)
    send_rc_channels(link_serial, ch, repeat=20, interval=0.05)
    print("[RC] Neutral/disarmed: R=1500 P=1500 T=1000 Y=1500 CH5=1000 CH6=1000")


def arm_fc(link_serial, throttle_test=1000, mode_value=MODE_ON_VALUE):
    ch = make_channels(1500, 1500, 1500, throttle_test, ARM_VALUE, mode_value)
    send_rc_channels(link_serial, ch, repeat=30, interval=0.05)
    print(f"[ARM] CH5={ARM_VALUE} CH6(mode)={mode_value} throttle={throttle_test}")


def disarm_fc(link_serial):
    ch = make_channels(1500, 1500, 1500, 1000, DISARM_VALUE, MODE_OFF_VALUE)
    send_rc_channels(link_serial, ch, repeat=30, interval=0.05)
    print(f"[DISARM] CH5={DISARM_VALUE} CH6(mode)={MODE_OFF_VALUE} throttle=1000")


def check_arm_status(link_serial):
    link_serial.write(build_msp_request(MSP_STATUS))
    resp = read_msp_response(link_serial, timeout=0.5)

    if resp and resp["code"] == MSP_STATUS and len(resp["payload"]) >= 8:
        flags = struct.unpack_from("<H", resp["payload"], 6)[0]
        return bool(flags & 0x0001)

    return None


# ============================================================
# MAIN
# ============================================================

def _try_make_hybrid():
    """YOLO+OpenCV hybrid; returns None if ultralytics/model unavailable."""
    if not USE_YOLO:
        return None
    try:
        from detection.hybrid_tracker import HybridYoloLockTracker

        return HybridYoloLockTracker(
            cv_kind=TRACKER_TYPE.lower(),  # type: ignore[arg-type]
            yolo_every_n=YOLO_EVERY_N,
            reacquire_iou=0.12,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"[YOLO] Hybrid unavailable ({exc}) — OpenCV-only tracking.")
        return None


def main():
    global select_mode, pending_roi, drag_start, drag_end

    print("=" * 80)
    print(" FPV MSP LOCK + FOLLOW (YOLO hybrid + yaw/pitch aim)")
    print("=" * 80)
    print(f"[LINK] {CONTROL_PORT} @ {CONTROL_BAUD}")
    print(f"[TRACKER] {TRACKER_TYPE} | YOLO={'ON' if USE_YOLO else 'OFF'} every {YOLO_EVERY_N}")

    try:
        link = serial.Serial(CONTROL_PORT, CONTROL_BAUD, timeout=0.02)
        time.sleep(2)
        print("[LINK] Connected.")
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {CONTROL_PORT}: {e}")
        print_ports()
        return

    send_neutral_disarmed(link)

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("[ERROR] Camera not opened. Try CAMERA_INDEX = 1 or 2.")
        disarm_fc(link)
        link.close()
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    window_name = "FPV MSP Lock Follow"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    hybrid = _try_make_hybrid()
    if hybrid is not None:
        try:
            hybrid.ensure_detector()
            print("[YOLO] Ready.")
        except Exception as exc:  # noqa: BLE001
            print(f"[YOLO] Load failed ({exc}) — OpenCV-only.")
            hybrid = None

    cv_tracker = None
    locked = False
    bbox = None
    lost_frames = 0
    track_source = "none"

    assist_enabled = False
    arm_requested = False
    mode_requested = False
    throttle_test = 1000

    controller = FPVFollowController(FPV_CFG)

    roll = RC_MID
    pitch = RC_MID
    yaw = RC_MID

    last_send = 0
    send_interval = 1.0 / SEND_HZ
    last_ctrl_log = 0.0
    ctrl_log_interval = 0.25

    print("\nControls:")
    print("L = mouse-drag lock | Y = YOLO auto-lock")
    print("E/D = assist on/off | A/X = arm/disarm+mode | M = mode")
    print("U/J = throttle | 0 = CH6=1900 | R = reset | S = status | Q = quit\n")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] Camera frame not received.")
                break

            frame_h, frame_w = frame.shape[:2]
            frame_cx = frame_w // 2
            frame_cy = frame_h // 2

            draw_crosshair(frame)

            status_text = "NO LOCK"
            error_x = 0
            error_y = 0
            dets = []

            # --- New manual ROI lock ---
            if pending_roi is not None:
                x, y, w, h = pending_roi
                if hybrid is not None:
                    hybrid.lock_xywh(frame, (x, y, w, h), label="manual")
                    cv_tracker = None
                else:
                    cv_tracker = create_tracker()
                    cv_tracker.init(frame, (x, y, w, h))
                bbox = (x, y, w, h)
                locked = True
                lost_frames = 0
                assist_enabled = True
                controller.reset()
                roll = pitch = yaw = RC_MID
                track_source = "lock"
                print(f"[LOCK] ROI locked: {bbox}")
                pending_roi = None

            # --- Track update ---
            tracking_ok = False
            if locked:
                if hybrid is not None and hybrid.locked:
                    result = hybrid.update(frame)
                    dets = result.detections
                    if result.ok and result.bbox_xywh is not None:
                        bbox = result.bbox_xywh
                        tracking_ok = bbox_area(bbox) >= MIN_BBOX_AREA
                        track_source = result.source
                    else:
                        tracking_ok = False
                        track_source = "lost"
                elif cv_tracker is not None:
                    tok, new_bbox = cv_tracker.update(frame)
                    if tok:
                        x, y, w, h = [int(v) for v in new_bbox]
                        bbox = (x, y, w, h)
                        tracking_ok = bbox_area(bbox) >= MIN_BBOX_AREA
                        track_source = "opencv"
                    else:
                        tracking_ok = False
                        track_source = "lost"

                if tracking_ok and bbox is not None:
                    lost_frames = 0
                    obj_cx, obj_cy = bbox_center(bbox)
                    error_x = obj_cx - frame_cx
                    error_y = obj_cy - frame_cy
                    status_text = f"LOCKED/{track_source.upper()}"

                    if assist_enabled:
                        roll, pitch, yaw = controller.update(
                            obj_cx, obj_cy, frame_w, frame_h
                        )
                        now_log = time.time()
                        if now_log - last_ctrl_log >= ctrl_log_interval:
                            last_ctrl_log = now_log
                            print(
                                f"[FOLLOW] err=({error_x:4d},{error_y:4d}) "
                                f"src={track_source:6s} "
                                f"R={roll} P={pitch} Y={yaw} T={throttle_test}"
                            )
                    else:
                        roll = pitch = yaw = RC_MID
                        controller.reset()

                    x, y, w, h = bbox
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)
                    cv2.circle(frame, (obj_cx, obj_cy), 6, (0, 255, 255), -1)
                    cv2.line(frame, (frame_cx, frame_cy), (obj_cx, obj_cy), (0, 255, 255), 2)
                else:
                    lost_frames += 1
                    status_text = f"TRACK LOST {lost_frames}/{MAX_LOST_FRAMES}"
                    roll, pitch, yaw = controller.fade_to_mid(0.88)

                    if lost_frames > MAX_LOST_FRAMES:
                        locked = False
                        cv_tracker = None
                        if hybrid is not None:
                            hybrid.reset()
                        bbox = None
                        assist_enabled = False
                        controller.reset()
                        roll = pitch = yaw = RC_MID
                        status_text = "LOCK DROPPED"
            else:
                roll = pitch = yaw = RC_MID
                # Preview YOLO boxes when unlocked (throttled)
                if hybrid is not None and (int(time.time() * 1000) // 200) % YOLO_EVERY_N == 0:
                    try:
                        dets = hybrid.detect_only(frame)
                    except Exception:  # noqa: BLE001
                        dets = []

            for d in dets:
                if locked and bbox is not None:
                    continue
                x1, y1, x2, y2 = map(int, d.as_xyxy())
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 180, 0), 1)
                if d.label:
                    cv2.putText(
                        frame, f"{d.label} {d.conf:.2f}", (x1, max(15, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 180, 0), 1,
                    )

            # Draw live selection rectangle
            if select_mode:
                cv2.putText(
                    frame,
                    "DRAG MOUSE BOX ON SUBJECT",
                    (20, 105),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 255),
                    2,
                )

                if tracking_drag and drag_start is not None and drag_end is not None:
                    x, y, w, h = normalize_roi(drag_start, drag_end)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)

            # MSP continuous send
            arm_value = ARM_VALUE if arm_requested else DISARM_VALUE
            mode_value = MODE_ON_VALUE if mode_requested else MODE_OFF_VALUE

            now = time.time()
            if now - last_send >= send_interval:
                last_send = now

                channels = make_channels(
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    throttle=throttle_test,
                    arm_value=arm_value,
                    mode_value=mode_value,
                )
                send_rc_channels(link, channels)

            # HUD
            cv2.putText(
                frame,
                f"{status_text}  ASSIST:{'ON' if assist_enabled else 'OFF'}  "
                f"ARM:{'ON' if arm_requested else 'OFF'}  MODE:{'ON' if mode_requested else 'OFF'}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 255) if locked else (0, 0, 255),
                2,
            )

            cv2.putText(
                frame,
                f"ERR X:{error_x} Y:{error_y}  SRC:{track_source}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255),
                2,
            )

            cv2.putText(
                frame,
                f"AETR R:{roll} P:{pitch} T:{throttle_test} Y:{yaw} CH5:{arm_value} CH6:{mode_value}",
                (20, frame_h - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )

            cv2.putText(
                frame,
                "L lock | Y yolo-lock | E/D assist | A/X arm | M mode | 0 CH6 | U/J thr | R reset | Q quit",
                (20, frame_h - 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                2,
            )

            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord("l"):
                select_mode = True
                assist_enabled = False
                controller.reset()
                print("[LOCK] Selection mode ON. Drag box on live feed.")

            elif key == ord("y"):
                if hybrid is None:
                    print("[YOLO] Not available.")
                else:
                    best = hybrid.lock_best(frame)
                    if best is not None:
                        locked = True
                        bbox = best.as_int_xywh()
                        lost_frames = 0
                        assist_enabled = True
                        controller.reset()
                        cv_tracker = None
                        print(f"[YOLO] Auto-locked {best.label} conf={best.conf:.2f} box={bbox}")
                    else:
                        print("[YOLO] No detection — try L drag lock.")

            elif key == ord("e"):
                if locked:
                    assist_enabled = True
                    controller.reset()
                    print("[ASSIST] ENABLED — FPV follow ON")
                else:
                    print("[WARN] Lock a target first (L drag or Y).")

            elif key == ord("d"):
                assist_enabled = False
                controller.reset()
                print("[ASSIST] DISABLED")

            elif key == ord("a"):
                arm_requested = True
                if MODE_FOLLOWS_ARM:
                    mode_requested = True
                assist_enabled = True
                controller.reset()
                arm_fc(
                    link,
                    throttle_test,
                    MODE_ON_VALUE if mode_requested else MODE_OFF_VALUE,
                )
                print(f"[ARM] Requested | MODE={'ON' if mode_requested else 'OFF'}")

            elif key == ord("x"):
                arm_requested = False
                if MODE_FOLLOWS_ARM:
                    mode_requested = False
                assist_enabled = False
                controller.reset()
                throttle_test = 1000
                disarm_fc(link)
                print("[DISARM] Requested. Mode OFF. Throttle reset 1000.")

            elif key == ord("m"):
                mode_requested = not mode_requested
                print(f"[MODE] Flight mode {'ON' if mode_requested else 'OFF'} (CH6)")

            elif key == ord("u"):
                throttle_test = int(clamp(throttle_test + 25, RC_MIN, RC_MAX))
                print(f"[THROTTLE] {throttle_test}")

            elif key == ord("j"):
                throttle_test = int(clamp(throttle_test - 25, RC_MIN, RC_MAX))
                print(f"[THROTTLE] {throttle_test}")

            elif key == ord("0"):
                mode_requested = True
                print(f"[MODE] CH6 set to {MODE_ON_VALUE}")

            elif key == ord("r"):
                locked = False
                cv_tracker = None
                if hybrid is not None:
                    hybrid.reset()
                bbox = None
                lost_frames = 0
                select_mode = False
                assist_enabled = False
                pending_roi = None
                track_source = "none"
                controller.reset()
                print("[RESET] Cleared tracker")

            elif key == ord("s"):
                status = check_arm_status(link)
                if status is True:
                    print("[FC] ARMED")
                elif status is False:
                    print("[FC] DISARMED")
                else:
                    print("[FC] Unknown/no MSP_STATUS response")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Exit.")

    finally:
        print("[SAFE] Disarm + neutral before exit.")
        try:
            disarm_fc(link)
            send_neutral_disarmed(link)
        except Exception as e:
            print(f"[SAFE] Error: {e}")

        try:
            cap.release()
        except Exception:
            pass

        try:
            link.close()
        except Exception:
            pass

        cv2.destroyAllWindows()
        print("[DONE]")


if __name__ == "__main__":
    main()
