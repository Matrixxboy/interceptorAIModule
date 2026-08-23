"""
Radxa ZERO 3W Headless Onboard Daemon for FPV Target Tracking.
Runs entirely without GUI components.
"""

import argparse
import time
import cv2
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


class OnboardTracker:
    def __init__(self, config_path: str):
        self.config_path = Path(config_path)
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

    def _load_config(self) -> SystemConfig:
        if not self.config_path.exists():
            print(f"[WARN] Config {self.config_path} not found. Generating default config.json.")
            cfg = SystemConfig()
            cfg.save_json(self.config_path)
            return cfg
        return SystemConfig.load_json(self.config_path)

    def _init_serial(self):
        # In a real onboard scenario, you'd specify the UART port in config.json.
        # Defaulting to /dev/ttyS2 for Radxa UART2 based on integration docs.
        # Here we'll try a list of fallbacks for dev purposes.
        ports = ["/dev/ttyS2", "/dev/ttyUSB0", "COM3"]
        for port in ports:
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.01)
                print(f"[INFO] Connected to FC on {port}")
                return
            except Exception:
                pass
        print("[WARN] Could not connect to Flight Controller UART.")

    def _init_camera(self):
        cam_idx = self.cfg.camera.camera_index
        # Try GStreamer pipeline for Rockchip/Linux hardware scaling first
        gst_pipeline = (
            f"v4l2src device=/dev/video{cam_idx} ! "
            f"video/x-raw, width={self.cfg.camera.frame_width}, height={self.cfg.camera.frame_height}, framerate={int(self.cfg.camera.target_fps)}/1 ! "
            "videoconvert ! appsink"
        )
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        # Fallback to standard V4L2 index
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0)
            
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.camera.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.camera.frame_height)
            fps = max(15.0, min(120.0, float(self.cfg.camera.target_fps or 60.0)))
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            print(f"[INFO] Camera initialized successfully at {fps} FPS.")
        else:
            print("[ERROR] Camera initialization failed.")

    def draw_osd_brackets(self, locked: bool):
        if not self.ser or not self.ser.is_open:
            return
        # Method 2: FC MSP Canvas OSD
        # Draw target brackets or idle text
        text = "[ TARGET LOCK ]" if locked else "    IDLE       "
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
        while True:
            frame_start = time.perf_counter()
            now = time.time()

            # 1. Capture Frame
            ok = False
            frame = None
            if self.cap and self.cap.isOpened():
                ok, frame = self.cap.read()
                
            if not ok or frame is None:
                print("[WARN] Camera frame dropped.")
                time.sleep(0.01)
                continue

            if now - self.last_rc_poll >= 0.02:
                self.last_rc_poll = now
                channels = self.poll_rc()
                if channels:
                    self.last_channels = channels
                    lock_sw, follow_sw = self.rc_manager.parse_channels(channels)
                else:
                    # RC Signal Loss / Timeout
                    lock_sw, follow_sw = False, False

                # 3. Target Detection & Tracking
                best_bbox = None
                has_target = False
                
                # Check current state *before* evaluating lock switch to know if we need to track
                if self.state_machine.state in [TrackingState.TARGET_LOCKED, TrackingState.FOLLOWING] or lock_sw:
                    if self.hybrid_tracker.locked:
                        res = self.hybrid_tracker.update(frame)
                        if res.ok and res.bbox_xywh:
                            best_bbox = res.bbox_xywh
                            has_target = True
                    else:
                        # Attempt lock if we just flipped the switch
                        best = self.hybrid_tracker.lock_best(frame)
                        if best:
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

            # Cap max FPS loop rate
            target_fps = max(15.0, min(120.0, float(self.cfg.camera.target_fps or 60.0)))
            frame_budget = 1.0 / target_fps
            elapsed = time.perf_counter() - frame_start
            if elapsed < frame_budget:
                time.sleep(frame_budget - elapsed)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.json", help="Path to centralized configuration JSON")
    args = parser.parse_args()
    
    try:
        daemon = OnboardTracker(args.config)
        daemon.run()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down onboard daemon.")
    except Exception as e:
        print(f"[FATAL] Unhandled exception: {e}")
        traceback.print_exc()
