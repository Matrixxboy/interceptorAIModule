"""
FPV graphical calibration — real stick movement + live follow.

1. Connect FC (MSP) + Open camera
2. Hold Yaw/Pitch buttons → craft moves (props OFF / ANGLE on)
3. Drag-lock a target → enable Live follow
4. If it turns the wrong way, Flip Yaw / Flip Pitch
5. Save JSON → used by main.py

Run:
    python calibration_fpv.py
"""

from __future__ import annotations

import os
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import cv2
import numpy as np
import serial
from serial.tools import list_ports

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from control.fpv_follow import FPVFollowController
from utils.calib_io import (
    DEFAULT_PATH,
    default_calibration,
    fpv_config_from_dict,
    load_calibration,
    save_calibration,
)

FEED_W = 900
FEED_H = 506

MSP_SET_RAW_RC = 200
NUM_CHANNELS = 16
RC_MIN, RC_MID, RC_MAX = 1000, 1500, 2000
ROLL_CH, PITCH_CH, THROTTLE_CH, YAW_CH = 0, 1, 2, 3
ARM_CH, MODE_CH = 4, 5
ARM_HIGH, ARM_LOW = 1800, 1000
MODE_HIGH, MODE_LOW = 1900, 1000

# Manual nudge strength while holding buttons (µs from mid)
NUDGE_US = 220
SEND_HZ = 50


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def build_msp_set_raw_rc(channels: list[int]) -> bytes:
    payload = struct.pack("<" + "H" * len(channels), *channels)
    size = len(payload)
    checksum = MSP_SET_RAW_RC ^ size
    for b in payload:
        checksum ^= b
    return b"$M<" + bytes([size, MSP_SET_RAW_RC]) + payload + bytes([checksum & 0xFF])


def make_channels(roll=1500, pitch=1500, yaw=1500, throttle=1000, arm=ARM_LOW, mode=MODE_LOW):
    ch = [1500] * NUM_CHANNELS
    ch[ROLL_CH] = int(_clamp(roll, RC_MIN, RC_MAX))
    ch[PITCH_CH] = int(_clamp(pitch, RC_MIN, RC_MAX))
    ch[THROTTLE_CH] = int(_clamp(throttle, RC_MIN, RC_MAX))
    ch[YAW_CH] = int(_clamp(yaw, RC_MIN, RC_MAX))
    ch[ARM_CH] = int(_clamp(arm, RC_MIN, RC_MAX))
    ch[MODE_CH] = int(_clamp(mode, RC_MIN, RC_MAX))
    return ch


def _bgr_to_photo(frame_bgr: np.ndarray) -> tk.PhotoImage:
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    header = f"P6 {w} {h} 255\n".encode("ascii")
    return tk.PhotoImage(data=header + rgb.tobytes())


def _fit_letterbox(frame: np.ndarray, tw: int, th: int):
    h, w = frame.shape[:2]
    scale = min(tw / w, th / h)
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((th, tw, 3), dtype=np.uint8)
    x0, y0 = (tw - nw) // 2, (th - nh) // 2
    canvas[y0 : y0 + nh, x0 : x0 + nw] = resized
    return canvas, scale, x0, y0


class CalibrationApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("FPV Calibration — Move sticks + live follow")
        self.root.geometry("1520x860")
        self.root.minsize(1320, 760)
        self.root.configure(bg="#1a1d23")

        self.calib = load_calibration()
        self.controller = FPVFollowController(fpv_config_from_dict(self.calib["fpv"]))

        self.cap = None
        self.cam_running = False
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.display_meta = (1.0, 0, 0)

        self.link: serial.Serial | None = None
        self.msp_running = False
        self.mode_on = False
        self.live_follow = False  # send follow sticks to FC when locked
        self.nudge_yaw = 0  # -1, 0, +1  (image-sense: + = toward right / yaw right intent)
        self.nudge_pitch = 0  # -1, 0, +1  (+ = toward up in image / pitch up intent)

        self.tracker = None
        self.locked = False
        self.bbox = None
        self.assist = True
        self.drag_start = None
        self._drag_end = None
        self.dragging = False

        self.roll = self.pitch = self.yaw = RC_MID
        self.error_x = self.error_y = 0
        self._photo = None
        self._hint = ""

        self._build_ui()
        self._refresh_labels()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(20, self._ui_tick)

    # ================================================================== UI
    def _build_ui(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        bg, card = "#1a1d23", "#242830"
        style.configure("TFrame", background=bg)
        style.configure("Card.TFrame", background=card)
        style.configure("TLabel", background=bg, foreground="#e8eaed", font=("Segoe UI", 10))
        style.configure("Head.TLabel", background=bg, foreground="#fff", font=("Segoe UI Semibold", 13))
        style.configure("Card.TLabel", background=card, foreground="#e8eaed", font=("Segoe UI", 9))
        style.configure("Value.TLabel", background=card, foreground="#7dd3fc", font=("Consolas", 10))
        style.configure("Status.TLabel", background=bg, foreground="#fbbf24", font=("Segoe UI", 10))
        style.configure("Hint.TLabel", background=bg, foreground="#86efac", font=("Segoe UI", 10))
        style.configure("Danger.TLabel", background=card, foreground="#fca5a5", font=("Segoe UI Semibold", 9))
        style.configure("Accent.TButton", font=("Segoe UI Semibold", 10), padding=7)
        style.configure("Move.TButton", font=("Segoe UI Semibold", 11), padding=10)
        style.configure("TButton", font=("Segoe UI", 9), padding=5)
        style.configure("TCheckbutton", background=card, foreground="#e8eaed", font=("Segoe UI", 9))
        style.configure("TLabelframe", background=card, foreground="#e8eaed")
        style.configure("TLabelframe.Label", background=card, foreground="#93c5fd", font=("Segoe UI Semibold", 10))
        style.configure("Vertical.TScrollbar", background=card)

        top = ttk.Frame(self.root)
        top.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # ---- Left feed ----
        left = ttk.Frame(top)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        ttk.Label(left, text="Camera — drag box on target to lock", style="Head.TLabel").pack(anchor="w")
        self.feed = tk.Label(
            left, width=FEED_W, height=FEED_H, bg="#0b0d10",
            highlightthickness=2, highlightbackground="#334155",
        )
        self.feed.pack(pady=6)
        self.feed.bind("<ButtonPress-1>", self._on_press)
        self.feed.bind("<B1-Motion>", self._on_drag)
        self.feed.bind("<ButtonRelease-1>", self._on_release)

        self.status_lbl = ttk.Label(left, text="1) Connect FC  2) Open camera  3) Hold move buttons", style="Status.TLabel")
        self.status_lbl.pack(anchor="w")
        self.hint_lbl = ttk.Label(left, text="", style="Hint.TLabel")
        self.hint_lbl.pack(anchor="w", pady=(2, 0))

        meters = ttk.Frame(left)
        meters.pack(fill=tk.X, pady=(6, 0))
        self.yaw_meter = self._meter(meters, "YAW stick")
        self.pitch_meter = self._meter(meters, "PITCH stick")

        # ---- Right scroll panel ----
        right_wrap = ttk.Frame(top, width=480)
        right_wrap.pack(side=tk.RIGHT, fill=tk.Y, padx=(12, 0))
        right_wrap.pack_propagate(False)

        canvas = tk.Canvas(right_wrap, bg=bg, highlightthickness=0, width=460)
        scroll = ttk.Scrollbar(right_wrap, orient="vertical", command=canvas.yview)
        right = ttk.Frame(canvas)
        right.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=right, anchor="nw", width=450)
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.bind_all("<MouseWheel>", lambda e: canvas.yview_scroll(int(-e.delta / 120), "units"))

        ttk.Label(right, text="FPV move + follow calibrate", style="Head.TLabel").pack(anchor="w", pady=(0, 6))

        # Link
        linkf = ttk.LabelFrame(right, text="1. Flight controller (MSP)", padding=8)
        linkf.pack(fill=tk.X, pady=4)
        ttk.Label(
            linkf,
            text="PROPS OFF. Connect USB/UART. Enable ANGLE on CH6 in INAV.",
            style="Danger.TLabel",
        ).pack(anchor="w")

        row = ttk.Frame(linkf, style="Card.TFrame")
        row.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(row, text="COM port", style="Card.TLabel").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=str(self.calib.get("control_port", "COM4")))
        ttk.Entry(row, textvariable=self.port_var, width=10).pack(side=tk.RIGHT)

        rowb = ttk.Frame(linkf, style="Card.TFrame")
        rowb.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(rowb, text="Connect FC", style="Accent.TButton", command=self.connect_fc).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4)
        )
        ttk.Button(rowb, text="Disconnect", command=self.disconnect_fc).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4)
        )
        ttk.Button(rowb, text="Ports", command=self.list_ports).pack(side=tk.LEFT, expand=True, fill=tk.X)

        rowm = ttk.Frame(linkf, style="Card.TFrame")
        rowm.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(rowm, text="ANGLE ON (CH6)", style="Accent.TButton", command=self.mode_on_cmd).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4)
        )
        ttk.Button(rowm, text="ANGLE OFF", command=self.mode_off_cmd).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )
        self.link_lbl = ttk.Label(linkf, text="FC: disconnected", style="Value.TLabel")
        self.link_lbl.pack(anchor="w", pady=(6, 0))

        # Camera
        camf = ttk.LabelFrame(right, text="2. Camera", padding=8)
        camf.pack(fill=tk.X, pady=4)
        row = ttk.Frame(camf, style="Card.TFrame")
        row.pack(fill=tk.X)
        ttk.Label(row, text="Camera index", style="Card.TLabel").pack(side=tk.LEFT)
        self.cam_var = tk.IntVar(value=int(self.calib["camera_index"]))
        ttk.Spinbox(row, from_=0, to=8, textvariable=self.cam_var, width=6).pack(side=tk.RIGHT)
        row2 = ttk.Frame(camf, style="Card.TFrame")
        row2.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(row2, text="Open camera", style="Accent.TButton", command=self.open_camera).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4)
        )
        ttk.Button(row2, text="Close camera", command=self.close_camera).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )

        # Manual move — the key feature
        movef = ttk.LabelFrame(right, text="3. Hold to MOVE craft (test directions)", padding=8)
        movef.pack(fill=tk.X, pady=4)
        ttk.Label(
            movef,
            text="Hold a button. Craft should move that way.\n"
                 "If yaw→ turns LEFT instead of RIGHT → Flip Yaw.\n"
                 "If pitch↑ noses DOWN instead of UP → Flip Pitch.",
            style="Card.TLabel",
            justify=tk.LEFT,
        ).pack(anchor="w")

        pad = ttk.Frame(movef, style="Card.TFrame")
        pad.pack(pady=8)
        self._hold_btn(pad, "▲  Pitch UP", 1, 0, 2).grid(row=0, column=1, padx=4, pady=4, sticky="ew")
        self._hold_btn(pad, "◀  Yaw LEFT", -1, 1, 0).grid(row=1, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(pad, text="CENTER", command=self.center_sticks).grid(row=1, column=1, padx=4, pady=4, sticky="ew")
        self._hold_btn(pad, "Yaw RIGHT  ▶", 1, 1, 2).grid(row=1, column=2, padx=4, pady=4, sticky="ew")
        self._hold_btn(pad, "▼  Pitch DOWN", -1, 2, 2).grid(row=2, column=1, padx=4, pady=4, sticky="ew")

        drow = ttk.Frame(movef, style="Card.TFrame")
        drow.pack(fill=tk.X, pady=(4, 0))
        ttk.Button(drow, text="Flip YAW dir", style="Accent.TButton", command=self.flip_yaw).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4)
        )
        ttk.Button(drow, text="Flip PITCH dir", style="Accent.TButton", command=self.flip_pitch).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )
        self.dir_lbl = ttk.Label(movef, text="", style="Value.TLabel")
        self.dir_lbl.pack(anchor="w", pady=(6, 0))

        # Live follow
        followf = ttk.LabelFrame(right, text="4. Live follow (lock target, watch craft aim)", padding=8)
        followf.pack(fill=tk.X, pady=4)
        ttk.Label(
            followf,
            text="Drag-lock target on feed. Enable Live follow.\n"
                 "Target on RIGHT → craft should yaw RIGHT toward it.\n"
                 "Target ABOVE → craft should pitch UP toward it.",
            style="Card.TLabel",
            justify=tk.LEFT,
        ).pack(anchor="w")

        self.live_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            followf,
            text="LIVE FOLLOW → send sticks to FC",
            variable=self.live_var,
            command=self._toggle_live,
        ).pack(anchor="w", pady=(6, 0))

        self.assist_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            followf, text="Compute follow (assist)", variable=self.assist_var, command=self._toggle_assist
        ).pack(anchor="w")

        frow = ttk.Frame(followf, style="Card.TFrame")
        frow.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(frow, text="Clear lock", command=self.clear_lock).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4)
        )
        ttk.Button(frow, text="Yaw wrong? Flip", command=self.flip_yaw).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4)
        )
        ttk.Button(frow, text="Pitch wrong? Flip", command=self.flip_pitch).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )

        self.tracker_var = tk.StringVar(value=str(self.calib.get("tracker_type", "CSRT")))
        ttk.Radiobutton(followf, text="CSRT", value="CSRT", variable=self.tracker_var).pack(anchor="w")
        ttk.Radiobutton(followf, text="KCF", value="KCF", variable=self.tracker_var).pack(anchor="w")

        # Gains
        gains = ttk.LabelFrame(right, text="5. Follow strength", padding=8)
        gains.pack(fill=tk.X, pady=4)
        self.yaw_kp_lbl = self._adj_row(gains, "Yaw Kp", self.dec_yaw_kp, self.inc_yaw_kp)
        self.pitch_kp_lbl = self._adj_row(gains, "Pitch Kp", self.dec_pitch_kp, self.inc_pitch_kp)
        self.max_yaw_lbl = self._adj_row(gains, "Max yaw", self.dec_max_yaw, self.inc_max_yaw)
        self.max_pitch_lbl = self._adj_row(gains, "Max pitch", self.dec_max_pitch, self.inc_max_pitch)
        self.lead_lbl = self._adj_row(gains, "Lead", self.dec_lead, self.inc_lead)
        self.dz_lbl = self._adj_row(gains, "Deadzone", self.dec_dz, self.inc_dz)
        self.nudge_lbl = self._adj_row(gains, "Nudge µs", self.dec_nudge, self.inc_nudge)
        self._nudge_us = NUDGE_US

        # Save
        savef = ttk.LabelFrame(right, text="6. Save", padding=8)
        savef.pack(fill=tk.X, pady=4)
        srow = ttk.Frame(savef, style="Card.TFrame")
        srow.pack(fill=tk.X)
        ttk.Button(srow, text="Save JSON", style="Accent.TButton", command=self.save).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4)
        )
        ttk.Button(srow, text="Reload", command=self.reload).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        ttk.Button(srow, text="Defaults", command=self.reset_defaults).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Label(savef, text=f"→ {DEFAULT_PATH.name}  (main.py loads this)", style="Card.TLabel").pack(
            anchor="w", pady=(6, 0)
        )

        self.rc_lbl = ttk.Label(right, text="RC  R:1500 P:1500 Y:1500", style="Value.TLabel")
        self.rc_lbl.pack(anchor="w", pady=(8, 0))
        self.err_lbl = ttk.Label(right, text="ERR  X:0 Y:0", style="Value.TLabel")
        self.err_lbl.pack(anchor="w")

    def _hold_btn(self, parent, text, sign, row, col):
        """sign: for yaw buttons use as yaw_sign; pitch as pitch_sign. Encoded via text."""
        btn = ttk.Button(parent, text=text, style="Move.TButton")
        if "Yaw LEFT" in text:
            btn.bind("<ButtonPress-1>", lambda e: self._nudge_yaw_set(-1))
            btn.bind("<ButtonRelease-1>", lambda e: self._nudge_yaw_set(0))
            btn.bind("<Leave>", lambda e: self._nudge_yaw_set(0))
        elif "Yaw RIGHT" in text:
            btn.bind("<ButtonPress-1>", lambda e: self._nudge_yaw_set(1))
            btn.bind("<ButtonRelease-1>", lambda e: self._nudge_yaw_set(0))
            btn.bind("<Leave>", lambda e: self._nudge_yaw_set(0))
        elif "Pitch UP" in text:
            btn.bind("<ButtonPress-1>", lambda e: self._nudge_pitch_set(1))
            btn.bind("<ButtonRelease-1>", lambda e: self._nudge_pitch_set(0))
            btn.bind("<Leave>", lambda e: self._nudge_pitch_set(0))
        elif "Pitch DOWN" in text:
            btn.bind("<ButtonPress-1>", lambda e: self._nudge_pitch_set(-1))
            btn.bind("<ButtonRelease-1>", lambda e: self._nudge_pitch_set(0))
            btn.bind("<Leave>", lambda e: self._nudge_pitch_set(0))
        return btn

    def _meter(self, parent, title):
        fr = ttk.Frame(parent)
        fr.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 10))
        ttk.Label(fr, text=title).pack(anchor="w")
        canvas = tk.Canvas(fr, height=20, bg="#0b0d10", highlightthickness=0)
        canvas.pack(fill=tk.X)
        return {"canvas": canvas}

    def _adj_row(self, parent, title, dec_cmd, inc_cmd):
        row = ttk.Frame(parent, style="Card.TFrame")
        row.pack(fill=tk.X, pady=2)
        ttk.Button(row, text="−", width=3, command=dec_cmd).pack(side=tk.LEFT)
        lbl = ttk.Label(row, text=f"{title}: —", style="Value.TLabel", width=20)
        lbl.pack(side=tk.LEFT, padx=6)
        ttk.Button(row, text="+", width=3, command=inc_cmd).pack(side=tk.LEFT)
        return lbl

    def _set_status(self, text: str) -> None:
        self.status_lbl.configure(text=text)

    def _set_hint(self, text: str) -> None:
        self.hint_lbl.configure(text=text)

    # ============================================================== MSP
    def list_ports(self) -> None:
        ports = list(list_ports.comports())
        if not ports:
            messagebox.showinfo("Ports", "No serial ports found.")
            return
        msg = "\n".join(f"{p.device}  —  {p.description}" for p in ports)
        messagebox.showinfo("Serial ports", msg)

    def connect_fc(self) -> None:
        self.disconnect_fc()
        port = self.port_var.get().strip()
        baud = int(self.calib.get("control_baud", 57600))
        try:
            self.link = serial.Serial(port, baud, timeout=0.02)
            time.sleep(1.5)
        except serial.SerialException as e:
            messagebox.showerror("FC", f"Could not open {port}:\n{e}")
            self.link = None
            return
        self.calib["control_port"] = port
        self.mode_on = False
        self.msp_running = True
        threading.Thread(target=self._msp_loop, daemon=True).start()
        # Neutral burst
        self._write_rc(RC_MID, RC_MID, RC_MID, 1000, ARM_LOW, MODE_LOW, repeat=10)
        self.link_lbl.configure(text=f"FC: connected {port} @ {baud}  |  ANGLE off")
        self._set_status(f"FC connected on {port} — press ANGLE ON, then hold move buttons")
        self._set_hint("Hold Yaw RIGHT — craft should turn right. If not, Flip Yaw.")

    def disconnect_fc(self) -> None:
        self.msp_running = False
        self.live_follow = False
        self.live_var.set(False)
        self.nudge_yaw = 0
        self.nudge_pitch = 0
        time.sleep(0.05)
        if self.link is not None:
            try:
                self._write_rc(RC_MID, RC_MID, RC_MID, 1000, ARM_LOW, MODE_LOW, repeat=15)
                self.link.close()
            except Exception:
                pass
        self.link = None
        self.link_lbl.configure(text="FC: disconnected")

    def mode_on_cmd(self) -> None:
        if self.link is None:
            messagebox.showwarning("FC", "Connect FC first.")
            return
        self.mode_on = True
        self.link_lbl.configure(text=f"FC: connected  |  ANGLE ON (CH6={MODE_HIGH})")
        self._set_hint("ANGLE on — hold move buttons (props OFF).")

    def mode_off_cmd(self) -> None:
        self.mode_on = False
        self.live_follow = False
        self.live_var.set(False)
        self.center_sticks()
        if self.link is not None:
            self.link_lbl.configure(text="FC: connected  |  ANGLE off")

    def _write_rc(self, roll, pitch, yaw, thr=1000, arm=ARM_LOW, mode=None, repeat=1):
        if self.link is None:
            return
        if mode is None:
            mode = MODE_HIGH if self.mode_on else MODE_LOW
        packet = build_msp_set_raw_rc(make_channels(roll, pitch, yaw, thr, arm, mode))
        try:
            for _ in range(repeat):
                self.link.write(packet)
        except Exception:
            pass

    def _msp_loop(self) -> None:
        interval = 1.0 / SEND_HZ
        while self.msp_running and self.link is not None:
            # Priority: hold-nudge > live follow > center
            if self.nudge_yaw != 0 or self.nudge_pitch != 0:
                # Physical sticks: RIGHT = yaw high, UP = pitch high (INAV Mode 2 style).
                # Flip Yaw/Pitch only changes LIVE FOLLOW mapping (image → stick).
                yaw = int(RC_MID + self.nudge_yaw * self._nudge_us)
                pitch = int(RC_MID + self.nudge_pitch * self._nudge_us)
                roll = RC_MID
                self.roll, self.pitch, self.yaw = roll, pitch, yaw
            elif self.live_follow and self.locked and self.assist:
                roll, pitch, yaw = self.roll, self.pitch, self.yaw
            else:
                roll = pitch = yaw = RC_MID
                if self.nudge_yaw == 0 and self.nudge_pitch == 0 and not self.live_follow:
                    self.roll = self.pitch = self.yaw = RC_MID

            mode = MODE_HIGH if self.mode_on else MODE_LOW
            self._write_rc(roll, pitch, yaw, 1000, ARM_LOW, mode, repeat=1)
            time.sleep(interval)

    def _nudge_yaw_set(self, v: int) -> None:
        self.nudge_yaw = v
        if v != 0:
            self.live_follow = False
            self.live_var.set(False)
            self._set_hint(
                "Holding yaw — watch craft. RIGHT should turn right. Wrong? Flip Yaw, then Save."
            )

    def _nudge_pitch_set(self, v: int) -> None:
        self.nudge_pitch = v
        if v != 0:
            self.live_follow = False
            self.live_var.set(False)
            self._set_hint(
                "Holding pitch — UP should raise nose. Wrong? Note: Flip Pitch is for FOLLOW (image), not this button."
            )

    def center_sticks(self) -> None:
        self.nudge_yaw = 0
        self.nudge_pitch = 0
        self.roll = self.pitch = self.yaw = RC_MID
        self.controller.reset()

    # =========================================================== camera
    def open_camera(self) -> None:
        self.close_camera()
        idx = int(self.cam_var.get())
        self.calib["camera_index"] = idx
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            messagebox.showerror("Camera", f"Could not open camera {idx}")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.calib["frame_width"]))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.calib["frame_height"]))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap = cap
        self.cam_running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()
        self._set_status(f"Camera {idx} open — drag box on target, or test move buttons")

    def close_camera(self) -> None:
        self.cam_running = False
        time.sleep(0.05)
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None
        self.clear_lock()

    def _capture_loop(self) -> None:
        while self.cam_running and self.cap is not None:
            ok, frame = self.cap.read()
            if ok:
                with self.frame_lock:
                    self.latest_frame = frame
            else:
                time.sleep(0.02)

    # ============================================================ mouse
    def _map_xy(self, event):
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return None
        scale, x0, y0 = self.display_meta
        fx = (event.x - x0) / max(scale, 1e-6)
        fy = (event.y - y0) / max(scale, 1e-6)
        h, w = frame.shape[:2]
        if fx < 0 or fy < 0 or fx >= w or fy >= h:
            return None
        return int(fx), int(fy)

    def _on_press(self, event) -> None:
        pt = self._map_xy(event)
        if pt is None:
            return
        self.dragging = True
        self.drag_start = pt
        self._drag_end = pt

    def _on_drag(self, event) -> None:
        if self.dragging:
            self._drag_end = self._map_xy(event)

    def _on_release(self, event) -> None:
        if not self.dragging or self.drag_start is None:
            return
        end = self._map_xy(event) or self._drag_end
        self.dragging = False
        if end is None:
            return
        x1, y1 = self.drag_start
        x2, y2 = end
        x, y = min(x1, x2), min(y1, y2)
        w, h = abs(x2 - x1), abs(y2 - y1)
        self.drag_start = self._drag_end = None
        if w < 12 or h < 12:
            self._set_status("Box too small")
            return
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return
        self._start_tracker(frame, (x, y, w, h))

    def _start_tracker(self, frame, xywh) -> None:
        kind = self.tracker_var.get().upper()
        tracker = None
        creators = []
        if kind == "CSRT":
            creators = [
                lambda: cv2.TrackerCSRT_create(),
                lambda: cv2.legacy.TrackerCSRT_create(),
            ]
        else:
            creators = [
                lambda: cv2.TrackerKCF_create(),
                lambda: cv2.legacy.TrackerKCF_create(),
            ]
        for fn in creators:
            try:
                tracker = fn()
                break
            except Exception:
                continue
        if tracker is None:
            messagebox.showerror("Tracker", "Install opencv-contrib-python")
            return
        tracker.init(frame, tuple(int(v) for v in xywh))
        self.tracker = tracker
        self.bbox = tuple(int(v) for v in xywh)
        self.locked = True
        self.controller = FPVFollowController(fpv_config_from_dict(self.calib["fpv"]))
        self._set_status(f"LOCKED {self.bbox} — enable LIVE FOLLOW to move craft toward target")
        self._set_hint("If craft turns away from target → Flip Yaw / Pitch, then Save.")

    def clear_lock(self) -> None:
        self.tracker = None
        self.locked = False
        self.bbox = None
        self.live_follow = False
        self.live_var.set(False)
        self.controller.reset()
        if self.nudge_yaw == 0 and self.nudge_pitch == 0:
            self.roll = self.pitch = self.yaw = RC_MID
        self.error_x = self.error_y = 0

    def _toggle_assist(self) -> None:
        self.assist = bool(self.assist_var.get())
        if not self.assist:
            self.controller.reset()

    def _toggle_live(self) -> None:
        self.live_follow = bool(self.live_var.get())
        if self.live_follow:
            if self.link is None:
                self.live_var.set(False)
                self.live_follow = False
                messagebox.showwarning("FC", "Connect FC first, then enable Live follow.")
                return
            if not self.mode_on:
                if messagebox.askyesno("ANGLE", "Enable ANGLE (CH6) for live follow?"):
                    self.mode_on_cmd()
            if not self.locked:
                self._set_hint("Live follow ON — now drag-lock a target on the feed.")
            else:
                self._set_hint("Live follow ON — craft should aim at locked target.")
            self.nudge_yaw = 0
            self.nudge_pitch = 0
        else:
            self.center_sticks()

    # ======================================================== calibrate
    def _rebuild_controller(self) -> None:
        self.controller = FPVFollowController(fpv_config_from_dict(self.calib["fpv"]))
        self._refresh_labels()

    def flip_yaw(self) -> None:
        self.calib["fpv"]["yaw_dir"] = float(self.calib["fpv"].get("yaw_dir", 1.0)) * -1.0
        self._rebuild_controller()
        self._set_hint(f"Yaw dir → {self.calib['fpv']['yaw_dir']:+.0f}  (live follow uses this)")

    def flip_pitch(self) -> None:
        self.calib["fpv"]["pitch_dir"] = float(self.calib["fpv"].get("pitch_dir", -1.0)) * -1.0
        self._rebuild_controller()
        self._set_hint(f"Pitch dir → {self.calib['fpv']['pitch_dir']:+.0f}  (live follow uses this)")

    def _bump(self, key, delta, lo, hi) -> None:
        cur = float(self.calib["fpv"].get(key, 0.0))
        self.calib["fpv"][key] = max(lo, min(hi, cur + delta))
        self._rebuild_controller()

    def inc_yaw_kp(self) -> None:
        self._bump("yaw_kp", 20, 50, 800)

    def dec_yaw_kp(self) -> None:
        self._bump("yaw_kp", -20, 50, 800)

    def inc_pitch_kp(self) -> None:
        self._bump("pitch_kp", 20, 50, 800)

    def dec_pitch_kp(self) -> None:
        self._bump("pitch_kp", -20, 50, 800)

    def inc_max_yaw(self) -> None:
        self._bump("max_yaw", 20, 80, 500)

    def dec_max_yaw(self) -> None:
        self._bump("max_yaw", -20, 80, 500)

    def inc_max_pitch(self) -> None:
        self._bump("max_pitch", 20, 80, 500)

    def dec_max_pitch(self) -> None:
        self._bump("max_pitch", -20, 80, 500)

    def inc_lead(self) -> None:
        self._bump("lead_s", 0.02, 0.0, 0.4)

    def dec_lead(self) -> None:
        self._bump("lead_s", -0.02, 0.0, 0.4)

    def inc_dz(self) -> None:
        self._bump("deadzone_norm", 0.005, 0.005, 0.12)

    def dec_dz(self) -> None:
        self._bump("deadzone_norm", -0.005, 0.005, 0.12)

    def inc_nudge(self) -> None:
        self._nudge_us = min(400, self._nudge_us + 20)
        self._refresh_labels()

    def dec_nudge(self) -> None:
        self._nudge_us = max(80, self._nudge_us - 20)
        self._refresh_labels()

    def _refresh_labels(self) -> None:
        fpv = self.calib["fpv"]
        self.dir_lbl.configure(
            text=f"follow yaw_dir={fpv['yaw_dir']:+.0f}   pitch_dir={fpv['pitch_dir']:+.0f}"
        )
        self.yaw_kp_lbl.configure(text=f"Yaw Kp: {fpv['yaw_kp']:.0f}")
        self.pitch_kp_lbl.configure(text=f"Pitch Kp: {fpv['pitch_kp']:.0f}")
        self.max_yaw_lbl.configure(text=f"Max yaw: {fpv['max_yaw']:.0f}")
        self.max_pitch_lbl.configure(text=f"Max pitch: {fpv['max_pitch']:.0f}")
        self.lead_lbl.configure(text=f"Lead: {fpv['lead_s']:.2f}")
        self.dz_lbl.configure(text=f"Deadzone: {fpv['deadzone_norm']:.3f}")
        self.nudge_lbl.configure(text=f"Nudge µs: {self._nudge_us}")

    # ======================================================== save/load
    def save(self) -> None:
        self.calib["camera_index"] = int(self.cam_var.get())
        self.calib["control_port"] = str(self.port_var.get()).strip() or "COM4"
        self.calib["tracker_type"] = self.tracker_var.get().upper()
        path = save_calibration(self.calib)
        self._set_status(f"Saved → {path}")
        messagebox.showinfo("Saved", f"Saved to:\n{path}\n\nmain.py loads this automatically.")

    def reload(self) -> None:
        self.calib = load_calibration()
        self.cam_var.set(int(self.calib["camera_index"]))
        self.port_var.set(str(self.calib.get("control_port", "COM4")))
        self.tracker_var.set(str(self.calib.get("tracker_type", "CSRT")))
        self._rebuild_controller()
        self._set_status("Reloaded calibration.json")

    def reset_defaults(self) -> None:
        if not messagebox.askyesno("Defaults", "Reset all values to defaults?"):
            return
        self.calib = default_calibration()
        self.cam_var.set(int(self.calib["camera_index"]))
        self.port_var.set(str(self.calib.get("control_port", "COM4")))
        self.tracker_var.set("CSRT")
        self._nudge_us = NUDGE_US
        self._rebuild_controller()

    # ============================================================ draw
    def _draw_meter(self, meter, value) -> None:
        c = meter["canvas"]
        c.update_idletasks()
        w = max(40, c.winfo_width())
        c.delete("all")
        c.create_rectangle(0, 4, w, 16, fill="#1e293b", outline="")
        c.create_line(w // 2, 2, w // 2, 18, fill="#64748b", width=2)
        x = int((value - 1000) / 1000.0 * w)
        color = "#38bdf8" if value >= 1500 else "#f472b6"
        c.create_rectangle(w // 2, 5, x, 15, fill=color, outline="")
        c.create_text(w - 4, 10, text=str(value), fill="#e2e8f0", anchor="e", font=("Consolas", 8))

    def _ui_tick(self) -> None:
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()

        if frame is not None:
            vis = frame.copy()
            h, w = vis.shape[:2]
            cx, cy = w // 2, h // 2
            cv2.drawMarker(vis, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 28, 2)
            dz = float(self.calib["fpv"].get("deadzone_norm", 0.02))
            dx, dy = int(w * 0.5 * dz), int(h * 0.5 * dz)
            cv2.rectangle(vis, (cx - dx, cy - dy), (cx + dx, cy + dy), (255, 255, 0), 1)

            if self.locked and self.tracker is not None:
                ok, bb = self.tracker.update(frame)
                if ok:
                    x, y, bw, bh = [int(v) for v in bb]
                    self.bbox = (x, y, bw, bh)
                    obj_cx, obj_cy = x + bw // 2, y + bh // 2
                    self.error_x = obj_cx - cx
                    self.error_y = obj_cy - cy
                    if self.assist and self.nudge_yaw == 0 and self.nudge_pitch == 0:
                        self.roll, self.pitch, self.yaw = self.controller.update(
                            float(obj_cx), float(obj_cy), w, h
                        )
                    color = (0, 255, 0) if self.live_follow else (0, 255, 255)
                    cv2.rectangle(vis, (x, y), (x + bw, y + bh), color, 2)
                    cv2.circle(vis, (obj_cx, obj_cy), 5, color, -1)
                    cv2.line(vis, (cx, cy), (obj_cx, obj_cy), color, 2)
                else:
                    self._set_status("TRACK LOST — redraw box")

            if self.dragging and self.drag_start and self._drag_end:
                cv2.rectangle(vis, self.drag_start, self._drag_end, (255, 0, 255), 2)

            mode = "LIVE" if self.live_follow else ("NUDGE" if (self.nudge_yaw or self.nudge_pitch) else "IDLE")
            cv2.putText(
                vis, f"{mode}  R:{self.roll} P:{self.pitch} Y:{self.yaw}",
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2,
            )
            cv2.putText(
                vis, f"err X:{self.error_x} Y:{self.error_y}",
                (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 255, 180), 2,
            )
            if self.mode_on:
                cv2.putText(vis, "ANGLE", (w - 100, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

            canvas, scale, x0, y0 = _fit_letterbox(vis, FEED_W, FEED_H)
            self.display_meta = (scale, x0, y0)
            self._photo = _bgr_to_photo(canvas)
            self.feed.configure(image=self._photo)

        self.rc_lbl.configure(text=f"RC  R:{self.roll}  P:{self.pitch}  Y:{self.yaw}")
        self.err_lbl.configure(text=f"ERR  X:{self.error_x}  Y:{self.error_y}")
        self._draw_meter(self.yaw_meter, self.yaw)
        self._draw_meter(self.pitch_meter, self.pitch)
        self.root.after(25, self._ui_tick)

    def _on_close(self) -> None:
        self.disconnect_fc()
        self.close_camera()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    CalibrationApp().run()


if __name__ == "__main__":
    main()
