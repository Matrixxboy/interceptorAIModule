"""
Radxa ZERO 3W Headless Onboard Daemon for FPV Target Tracking.
Runs entirely without GUI components.
"""

import argparse
import threading
import time
import cv2
import numpy as np
import serial
import traceback
from pathlib import Path

from config import SystemConfig
from core.state_machine import TargetTrackingStateMachine, TrackingState
from control.rc_manager import RCManager
from detection.hybrid_tracker import HybridYoloLockTracker
from control.fpv_follow import FPVFollowController
from control.msp_link import (
    build_msp_request,
    read_msp_response,
    parse_msp_rc,
    build_msp_displayport_draw,
    MSP_RC,
    make_rc_channels
)


class ThreadedCameraReader:
    """Threaded camera reader to continuously empty OS video buffers for 0ms real-time camera capture."""

    def __init__(self, cap: cv2.VideoCapture) -> None:
        self.cap = cap
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self.ret, self.frame = self.cap.read()
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        self.thread.start()

    def _update_loop(self) -> None:
        while self.running:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    with self.lock:
                        self.ret, self.frame = ret, frame
                else:
                    time.sleep(0.005)
            else:
                time.sleep(0.01)

    def isOpened(self) -> bool:
        return self.cap is not None and self.cap.isOpened()

    def read(self) -> tuple[bool, np.ndarray | None]:
        with self.lock:
            if self.frame is not None:
                return self.ret, self.frame.copy()
            return self.ret, self.frame


class OnboardTracker:
    def __init__(self, config_path: str, show_window: bool = False):
        self.config_path = Path(config_path)
        self.show_window = show_window
        self.cfg = self._load_config()
        self.rc_manager = RCManager(self.cfg)
        self.state_machine = TargetTrackingStateMachine()
        
        self.hybrid_tracker = HybridYoloLockTracker(
            det_cfg=self.cfg.detection,
            tracker_cfg=self.cfg.tracker,
        )
        self.controller = FPVFollowController(self.cfg)

        self.ser = None
        self._init_serial()
        self.cap = None
        self._init_camera()

        self.last_rc_poll = 0.0
        self.last_osd_draw = 0.0
        self.last_channels = None
        
        # Keyboard switch overrides for test mode
        self.kb_lock_sw = False
        self.kb_follow_sw = False
        self.fps_counter = 0.0
        self._frame_count = 0
        self._cached_candidates = []

    def _load_config(self) -> SystemConfig:
        if not self.config_path.exists():
            print(f"[WARN] Config {self.config_path} not found. Generating default config.json.")
            cfg = SystemConfig()
            cfg.save_json(self.config_path)
            return cfg
        return SystemConfig.load_json(self.config_path)

    def _init_serial(self):
        # Defaulting to /dev/ttyS2 for Radxa ZERO 3 40-pin header UART2 (pins 8/10)
        # Fallback list covers Linux SBC device nodes and dev USB bridges.
        ports = [
            "/dev/ttyS2",
            "/dev/ttyS0",
            "/dev/ttyS4",
            "/dev/ttyFIQ0",
            "/dev/ttyUSB0",
            "/dev/ttyACM0",
            "COM3",
            "COM4",
        ]
        for port in ports:
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.01)
                print(f"[INFO] Connected to FC on {port}")
                return
            except Exception:
                pass
        print("[WARN] Could not connect to Flight Controller UART.")
        print("       On Radxa ZERO 3, verify UART2 wiring on GPIO Pins 8 (TX) / 10 (RX) & user dialout group permissions.")

    def _init_camera(self):
        cam_idx = self.cfg.camera.camera_index
        w = self.cfg.camera.frame_width
        h = self.cfg.camera.frame_height
        target_fps = int(self.cfg.camera.target_fps or 60)

        # GStreamer pipelines for Rockchip/Linux hardware & MS2109 USB capture dongles
        gst_pipelines = [
            # 1. MJPEG pipeline (Ideal for MS2109 USB capture & high FPS)
            (
                f"v4l2src device=/dev/video{cam_idx} io-mode=2 ! "
                f"image/jpeg, width={w}, height={h}, framerate={target_fps}/1 ! "
                "jpegdec ! videoconvert ! appsink"
            ),
            # 2. Raw YUV pipeline
            (
                f"v4l2src device=/dev/video{cam_idx} ! "
                f"video/x-raw, width={w}, height={h}, framerate={target_fps}/1 ! "
                "videoconvert ! appsink"
            ),
        ]

        for pipe in gst_pipelines:
            try:
                self.cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
                if self.cap and self.cap.isOpened():
                    self.cam_reader = ThreadedCameraReader(self.cap)
                    print(f"[INFO] Camera initialized via GStreamer on /dev/video{cam_idx} (Threaded 0ms Buffer).")
                    return
            except Exception:
                pass

        # Fallback to standard V4L2 index with MJPG FOURCC
        for idx in [cam_idx, 0, 1, 2]:
            self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if self.cap and self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                fps = max(15.0, min(120.0, float(target_fps)))
                self.cap.set(cv2.CAP_PROP_FPS, fps)
                self.cam_reader = ThreadedCameraReader(self.cap)
                print(f"[INFO] Camera initialized via V4L2 on index {idx} ({w}x{h} @ {fps} FPS, Threaded 0ms Buffer).")
                return

        print("[ERROR] Camera initialization failed across all GStreamer and V4L2 devices.")

    def draw_osd_brackets(self, locked: bool):
        if not self.ser or not self.ser.is_open:
            return
        tid = getattr(self.hybrid_tracker, "target_id", -1)
        if locked and tid > 0:
            text = f" LOCK #{tid} "
        elif locked:
            text = "[ TARGET LOCK ]"
        else:
            text = "    IDLE       "
        try:
            # Row 5, Col 10
            payload = build_msp_displayport_draw(5, 10, text)
            self.ser.write(payload)
        except Exception as e:
            print(f"[ERROR] OSD Draw failed: {e}")

    def poll_rc(self) -> list[int] | None:
        if not self.ser or not self.ser.is_open:
            return None
            
        # Send MSP_RC request
        try:
            self.ser.write(build_msp_request(MSP_RC))
            res = read_msp_response(self.ser, timeout=0.02)
            if res:
                cmd, payload = res
                if cmd == MSP_RC:
                    return parse_msp_rc(payload)
        except Exception:
            pass
        return None

    def run(self):
        print("[INFO] Onboard tracking daemon started.")
        if self.show_window:
            print("[INFO] Test Mode: Live Camera Display Window Active.")
            print("       Keyboard Controls: [L] Lock Toggle | [F] Follow Toggle | [R] Reset | [Q/ESC] Quit")

        last_time = time.perf_counter()

        while True:
            self._frame_count += 1
            frame_start = time.perf_counter()
            now = time.time()
            
            # FPS Calculation
            dt = frame_start - last_time
            last_time = frame_start
            if dt > 0:
                self.fps_counter = 0.9 * self.fps_counter + 0.1 * (1.0 / dt)

            # 1. Capture Frame (Threaded 0ms Real-Time Buffer)
            ok = False
            frame = None
            if hasattr(self, "cam_reader") and self.cam_reader and self.cam_reader.isOpened():
                ok, frame = self.cam_reader.read()
            elif self.cap and self.cap.isOpened():
                ok, frame = self.cap.read()
                
            if not ok or frame is None:
                print("[WARN] Camera frame dropped.")
                time.sleep(0.01)
                continue

            # 2. RC & Keyboard Control Polling
            rc_lock_sw, rc_follow_sw = False, False
            if now - self.last_rc_poll >= 0.02:
                self.last_rc_poll = now
                channels = self.poll_rc()
                if channels:
                    self.last_channels = channels
                    rc_lock_sw, rc_follow_sw = self.rc_manager.parse_channels(channels)

            # Merge physical RC switches with Keyboard test switches
            lock_sw = rc_lock_sw or self.kb_lock_sw
            follow_sw = rc_follow_sw or self.kb_follow_sw

            fh, fw = frame.shape[:2]
            cx_ref, cy_ref = fw // 2, fh // 2

            # 3. Target Detection & OpenCV Pattern Tracking
            best_bbox = None
            has_target = False
            candidate_boxes = []

            if self.hybrid_tracker.locked:
                # OpenCV Pattern Lock (LK Optical Flow + Scale Template) updates target on 100% of frames at 60+ FPS
                res = self.hybrid_tracker.update(frame)
                candidate_boxes = res.detections or getattr(self, "_cached_candidates", [])
                if res.ok and res.bbox_xywh:
                    best_bbox = res.bbox_xywh
                    has_target = True
            else:
                # When unlocked: run YOLO detection every 4 frames (or on lock trigger) to keep camera feed smooth
                if self._frame_count % 4 == 0 or lock_sw:
                    self._cached_candidates = self.hybrid_tracker.detect_only(frame)
                candidate_boxes = getattr(self, "_cached_candidates", [])
                
                # Lock onto candidate nearest to center crosshair when Channel 7 / L key is active
                if lock_sw:
                    best = self.hybrid_tracker.lock_nearest_to_center(frame, center_xy=(cx_ref, cy_ref))
                    if best or self.hybrid_tracker._bbox:
                        best_bbox = self.hybrid_tracker._bbox
                        has_target = True

            # 4. State Machine Update
            current_state = self.state_machine.update(lock_sw, follow_sw, has_target)

            # 5. Handle State Actions
            if current_state == TrackingState.IDLE:
                self.hybrid_tracker.reset()
                self.controller.reset()
                
            elif current_state == TrackingState.TARGET_LOCKED:
                self.controller.reset()  # No commands sent, but keep tracking
                
            elif current_state == TrackingState.FOLLOWING:
                # Execute active PID tracking commands
                h, w = frame.shape[:2]
                roll, pitch, yaw, throttle = self.controller.update(
                    best_bbox, w, h, base_throttle=1500
                )
                # Send commands to FC, preserving the pilot's AUX switches
                if self.ser and self.ser.is_open and self.last_channels:
                    rc_payload = make_rc_channels(
                        roll=roll, pitch=pitch, yaw=yaw, throttle=throttle, 
                        base_channels=self.last_channels,
                        roll_ch=self.cfg.rc_control.roll_channel,
                        pitch_ch=self.cfg.rc_control.pitch_channel,
                        throttle_ch=self.cfg.rc_control.throttle_channel,
                        yaw_ch=self.cfg.rc_control.yaw_channel,
                    )
                    from control.msp_link import build_msp_set_raw_rc
                    self.ser.write(build_msp_set_raw_rc(rc_payload))

            # 6. Canvas OSD Update (Method 2)
            if now - self.last_osd_draw >= 0.1:  # 10Hz OSD refresh
                self.last_osd_draw = now
                self.draw_osd_brackets(locked=has_target)

            # 7. GUI Display & Keyboard Test Controls
            if self.show_window:
                vis_frame = frame.copy()

                # Draw Center Crosshair
                cv2.drawMarker(vis_frame, (cx_ref, cy_ref), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)

                # Draw ALL Candidate Boxes (Yellow/Cyan)
                for cand in candidate_boxes:
                    cv2.rectangle(vis_frame, (cand.x1, cand.y1), (cand.x2, cand.y2), (0, 255, 255), 1)
                    conf_pct = int(cand.conf * 100)
                    cv2.putText(vis_frame, f"{cand.label} {conf_pct}%", (cand.x1, max(15, cand.y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

                # Draw LOCKED Target Box & Guidance Vector (Green / Red)
                if has_target and best_bbox:
                    bx, by, bw, bh = best_bbox
                    tcx, tcy = bx + bw // 2, by + bh // 2
                    color = (0, 255, 0) if current_state == TrackingState.FOLLOWING else (0, 255, 255)
                    tid = getattr(self.hybrid_tracker, "target_id", 1)
                    
                    # Highlight locked box with thick border
                    cv2.rectangle(vis_frame, (bx, by), (bx + bw, by + bh), color, 3)
                    # Center dot on locked target
                    cv2.circle(vis_frame, (tcx, tcy), 6, (0, 0, 255), -1)
                    # Vector line connecting screen center crosshair to target center
                    cv2.line(vis_frame, (cx_ref, cy_ref), (tcx, tcy), (0, 0, 255), 2)
                    
                    # Offset & Target ID text
                    dx, dy = tcx - cx_ref, tcy - cy_ref
                    cv2.putText(vis_frame, f"[ TARGET #{tid} ] dx:{dx} dy:{dy}", (bx, max(25, by - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

                # Top HUD Banner (Config Channels Mapping Display)
                lock_ch_num = self.cfg.rc_control.lock_channel + 1
                follow_ch_num = self.cfg.rc_control.follow_channel + 1
                state_str = str(current_state).replace("TrackingState.", "")
                
                hud_top = f"STATE: {state_str} | FPS: {self.fps_counter:.1f} | CH{lock_ch_num} LOCK: {'ON' if lock_sw else 'OFF'} | CH{follow_ch_num} FOLLOW: {'ON' if follow_sw else 'OFF'}"
                cv2.rectangle(vis_frame, (0, 0), (fw, 36), (0, 0, 0), -1)
                cv2.putText(vis_frame, hud_top, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

                # Bottom Controls Help Bar
                hud_bot = f"[L] Toggle CH{lock_ch_num} Lock  |  [F] Toggle CH{follow_ch_num} Follow  |  [R] Reset  |  [Q/ESC] Quit"
                cv2.rectangle(vis_frame, (0, fh - 30), (fw, fh), (0, 0, 0), -1)
                cv2.putText(vis_frame, hud_bot, (10, fh - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

                cv2.imshow("FPV Interceptor - Live Camera Feed (Test Mode)", vis_frame)

                # Handle Keyboard Inputs
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):  # Q or ESC
                    print("[INFO] Exit key pressed. Shutting down...")
                    break
                elif key in (ord("l"), ord("L")):
                    self.kb_lock_sw = not self.kb_lock_sw
                    print(f"[TEST KEY] Keyboard Lock Switch (CH{lock_ch_num}) toggled -> {'ON' if self.kb_lock_sw else 'OFF'}")
                elif key in (ord("f"), ord("F"), ord("e"), ord("E")):
                    self.kb_follow_sw = not self.kb_follow_sw
                    print(f"[TEST KEY] Keyboard Follow Switch (CH{follow_ch_num}) toggled -> {'ON' if self.kb_follow_sw else 'OFF'}")
                elif key in (ord("r"), ord("R")):
                    self.kb_lock_sw = False
                    self.kb_follow_sw = False
                    self.hybrid_tracker.reset()
                    self.controller.reset()
                    print("[TEST KEY] Tracker & State Reset.")

            # Cap max FPS loop rate
            target_fps = max(15.0, min(120.0, float(self.cfg.camera.target_fps or 60.0)))
            frame_budget = 1.0 / target_fps
            elapsed = time.perf_counter() - frame_start
            if elapsed < frame_budget:
                time.sleep(frame_budget - elapsed)

        if self.show_window:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.json", help="Path to centralized configuration JSON")
    parser.add_argument("--show", action="store_true", default=True, help="Display live camera window with HUD and keyboard test controls")
    args = parser.parse_args()
    
    try:
        daemon = OnboardTracker(args.config, show_window=args.show)
        daemon.run()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down onboard daemon.")
    except Exception as e:
        print(f"[FATAL] Unhandled exception: {e}")
        traceback.print_exc()
