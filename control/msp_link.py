"""INAV / Betaflight MSP (MultiWii Serial Protocol) RC Transmission & Telemetry Module."""

from __future__ import annotations

import struct
import time
from typing import Sequence
import serial
from serial.tools import list_ports

# MSP Command Identifiers
MSP_IDENT = 100
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_RC = 105
MSP_STATUS_EX = 150
MSP_DISPLAYPORT = 182
MSP_SET_RAW_RC = 200

NUM_CHANNELS = 16

RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000

ROLL_CH = 0
PITCH_CH = 1
THROTTLE_CH = 2
YAW_CH = 3
ARM_CH = 4   # AUX1 (Index 4)
MODE_CH = 5  # AUX2 (Index 5)

ARM_HIGH = 1800
ARM_LOW = 1000
MODE_HIGH = 1900
MODE_LOW = 1000

ARMING_DISABLE_FLAG_NAMES = [
    "NO_GYRO",
    "FAILSAFE",
    "RX_FAILSAFE",
    "BAD_RX_RECOVERY",
    "BOXFAILSAFE",
    "RUNAWAY_TAKEOFF",
    "CRASH_DETECTED",
    "THROTTLE",
    "ANGLE",
    "BOOT_GRACE",
    "NOPROFILES",
    "LOAD",
    "CALIBRATING",
    "CLI",
    "CMS",
    "BST",
    "MSP",
    "PARALYZE",
    "GPS",
    "RESCUE",
]


def list_serial_ports() -> list[tuple[str, str]]:
    """Return list of available (device_path, description)."""
    ports = []
    for p in list_ports.comports():
        ports.append((p.device, f"{p.device} ({p.description})"))
    return ports


def clamp(val: float, lo: float, hi: float) -> int:
    return int(max(lo, min(hi, val)))


def build_msp_request(cmd_id: int) -> bytes:
    """Build $M< payload for querying MSP telemetry."""
    size = 0
    checksum = (size ^ cmd_id) & 0xFF
    return b"$M<" + bytes([size, cmd_id, checksum])


def build_msp_set_raw_rc(channels: Sequence[int]) -> bytes:
    """Build $M< payload for MSP_SET_RAW_RC (200)."""
    if len(channels) != NUM_CHANNELS:
        ch_list = list(channels) + [RC_MID] * (NUM_CHANNELS - len(channels))
        channels = ch_list[:NUM_CHANNELS]

    safe_channels = [clamp(c, RC_MIN, RC_MAX) for c in channels]
    payload = struct.pack("<" + "H" * NUM_CHANNELS, *safe_channels)
    size = len(payload)

    checksum = MSP_SET_RAW_RC ^ size
    for b in payload:
        checksum ^= b
    checksum &= 0xFF

    return b"$M<" + bytes([size, MSP_SET_RAW_RC]) + payload + bytes([checksum])


def read_msp_response(ser: serial.Serial, timeout: float = 0.1) -> tuple[int, bytes] | None:
    """Parse incoming $M> response header & payload from FC."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if ser.in_waiting >= 6:
            header = ser.read(3)
            if header == b"$M>":
                size = ord(ser.read(1))
                cmd = ord(ser.read(1))
                payload = ser.read(size)
                chk = ord(ser.read(1))
                expected_chk = size ^ cmd
                for b in payload:
                    expected_chk ^= b
                expected_chk &= 0xFF
                if chk == expected_chk:
                    return cmd, payload
        else:
            time.sleep(0.005)
    return None


def parse_msp_attitude(payload: bytes) -> dict | None:
    """Parse MSP_ATTITUDE (108): Roll, Pitch, Yaw in degrees."""
    if len(payload) >= 6:
        angx, angy, heading = struct.unpack("<h h h", payload[:6])
        return {
            "roll_deg": angx / 10.0,
            "pitch_deg": angy / 10.0,
            "yaw_deg": float(heading),
        }
    return None


def parse_msp_analog(payload: bytes) -> dict | None:
    """Parse MSP_ANALOG (110): Battery Voltage, mAh, RSSI, Current."""
    if len(payload) >= 7:
        vbat, power_sum, rssi, amperage = struct.unpack("<B H H H", payload[:7])
        return {
            "vbat_volts": vbat / 10.0,
            "mah_drawn": power_sum,
            "rssi": rssi,
            "amperage_a": amperage / 100.0,
        }
    return None


def parse_msp_rc(payload: bytes) -> list[int] | None:
    """Parse MSP_RC (105): RC channel values (16 channels max)."""
    if len(payload) >= 2:
        num_channels = len(payload) // 2
        channels = list(struct.unpack("<" + "H" * num_channels, payload[:num_channels * 2]))
        return channels
    return None


def build_msp_displayport_draw(row: int, col: int, text: str) -> bytes:
    """Build $M< payload for MSP_DISPLAYPORT (182) to draw text."""
    # Subcmd 3 = DRAW_STRING
    attr = 0  # Normal text
    payload = struct.pack("<B B B B", 3, row, col, attr) + text.encode("ascii", "ignore")
    size = len(payload)
    
    checksum = size ^ MSP_DISPLAYPORT
    for b in payload:
        checksum ^= b
    checksum &= 0xFF
    return b"$M<" + bytes([size, MSP_DISPLAYPORT]) + payload + bytes([checksum])


def build_msp_displayport_clear() -> bytes:
    """Build MSP_DISPLAYPORT subcmd 2 = CLEAR_SCREEN."""
    payload = struct.pack("<B", 2)
    size = len(payload)
    checksum = (size ^ MSP_DISPLAYPORT ^ payload[0]) & 0xFF
    return b"$M<" + bytes([size, MSP_DISPLAYPORT]) + payload + bytes([checksum])


def build_msp_displayport_draw_screen() -> bytes:
    """Build MSP_DISPLAYPORT subcmd 4 = DRAW_SCREEN (flush OSD buffer to display)."""
    payload = struct.pack("<B", 4)
    size = len(payload)
    checksum = (size ^ MSP_DISPLAYPORT ^ payload[0]) & 0xFF
    return b"$M<" + bytes([size, MSP_DISPLAYPORT]) + payload + bytes([checksum])


def draw_osd_target_box(
    ser,
    bbox_xywh: tuple[int, int, int, int] | None,
    frame_w: int,
    frame_h: int,
    locked: bool,
    target_id: int = -1,
    osd_cols: int = 30,
    osd_rows: int = 16,
) -> None:
    """Draw target bounding box corners and status on FPV goggle OSD via MSP DisplayPort.

    Converts pixel coordinates to OSD character grid and draws corner brackets
    plus lock/follow status text. Works with INAV/Betaflight MSP DisplayPort over UART.

    Args:
        ser: serial.Serial connected to FC UART (Pin 8 TX / Pin 10 RX on Radxa ZERO 3).
        bbox_xywh: Target bounding box in pixel coordinates (x, y, w, h), or None if no target.
        frame_w: Camera frame width in pixels.
        frame_h: Camera frame height in pixels.
        locked: Whether a target is currently locked.
        target_id: Persistent target ID number.
        osd_cols: OSD character grid columns (30 for PAL analog, 50 for HD).
        osd_rows: OSD character grid rows (16 for PAL analog, 18 for HD).
    """
    if ser is None or not ser.is_open:
        return

    try:
        # 1. Clear OSD canvas
        ser.write(build_msp_displayport_clear())

        # 2. Draw center crosshair on OSD
        cx_osd = osd_cols // 2
        cy_osd = osd_rows // 2
        ser.write(build_msp_displayport_draw(cy_osd, cx_osd - 1, "-+-"))
        ser.write(build_msp_displayport_draw(cy_osd - 1, cx_osd, "|"))
        ser.write(build_msp_displayport_draw(cy_osd + 1, cx_osd, "|"))

        # 3. Draw target bounding box corners on OSD grid
        if locked and bbox_xywh is not None:
            bx, by, bw, bh = bbox_xywh

            # Convert pixel coords to OSD grid coords
            col_l = max(0, min(osd_cols - 2, int(bx / max(1, frame_w) * osd_cols)))
            row_t = max(0, min(osd_rows - 2, int(by / max(1, frame_h) * osd_rows)))
            col_r = max(col_l + 2, min(osd_cols - 1, int((bx + bw) / max(1, frame_w) * osd_cols)))
            row_b = max(row_t + 1, min(osd_rows - 1, int((by + bh) / max(1, frame_h) * osd_rows)))

            # Draw 4 corners of bounding box
            ser.write(build_msp_displayport_draw(row_t, col_l, "/"))      # Top-Left
            ser.write(build_msp_displayport_draw(row_t, col_r, "\\"))     # Top-Right
            ser.write(build_msp_displayport_draw(row_b, col_l, "\\"))     # Bottom-Left
            ser.write(build_msp_displayport_draw(row_b, col_r, "/"))      # Bottom-Right

            # Draw top/bottom edge markers
            edge_w = col_r - col_l
            if edge_w > 3:
                top_bar = "-" * min(edge_w - 1, 10)
                ser.write(build_msp_displayport_draw(row_t, col_l + 1, top_bar))
                ser.write(build_msp_displayport_draw(row_b, col_l + 1, top_bar))

            # Draw target ID label above box
            if target_id > 0:
                label = f"TGT#{target_id}"
            else:
                label = "TARGET"
            label_col = max(0, min(osd_cols - len(label), col_l))
            label_row = max(0, row_t - 1)
            ser.write(build_msp_displayport_draw(label_row, label_col, label))

        # 4. Draw status bar at bottom
        if locked and target_id > 0:
            status = f"LOCK #{target_id} ACTIVE"
        elif locked:
            status = "[ TARGET LOCKED ]"
        else:
            status = "SCANNING..."
        status_col = max(0, (osd_cols - len(status)) // 2)
        ser.write(build_msp_displayport_draw(osd_rows - 1, status_col, status))

        # 5. Flush OSD buffer to display
        ser.write(build_msp_displayport_draw_screen())

    except Exception:
        pass


def parse_msp_altitude(payload: bytes) -> dict | None:
    """Parse MSP_ALTITUDE (109): Altitude in meters and vario."""
    if len(payload) >= 6:
        alt_cm, vario = struct.unpack("<i h", payload[:6])
        return {
            "alt_m": alt_cm / 100.0,
            "vario_cms": vario,
        }
    return None


def parse_msp_status(payload: bytes) -> dict | None:
    """Parse MSP_STATUS (101): Cycle time, i2c errors, sensor flags."""
    if len(payload) >= 11:
        cycle_time, i2c_err, sensors, flag, current_set = struct.unpack("<H H H I B", payload[:11])
        return {
            "cycle_time_us": cycle_time,
            "i2c_errors": i2c_err,
            "sensors": sensors,
            "armed": bool(flag & 0x01),
            "arming_disable_reasons": [],
        }
    return None


def parse_msp_status_ex(payload: bytes) -> dict | None:
    """Parse MSP_STATUS_EX (150): Status with active arming disable flags."""
    if len(payload) >= 15:
        cycle_time, i2c_err, sensors, flag, current_set, arm_flags = struct.unpack("<H H H I B I", payload[:15])
        active_reasons = []
        for bit_i in range(32):
            if (arm_flags >> bit_i) & 0x01:
                name = ARMING_DISABLE_FLAG_NAMES[bit_i] if bit_i < len(ARMING_DISABLE_FLAG_NAMES) else f"FLAG_{bit_i}"
                active_reasons.append(name)
        return {
            "cycle_time_us": cycle_time,
            "i2c_errors": i2c_err,
            "sensors": sensors,
            "armed": bool(flag & 0x01),
            "arming_disabled_raw": arm_flags,
            "arming_disable_reasons": active_reasons,
        }
    return parse_msp_status(payload)


def make_rc_channels(
    roll: int = RC_MID,
    pitch: int = RC_MID,
    yaw: int = RC_MID,
    throttle: int = RC_MIN,
    base_channels: list[int] | None = None,
    roll_ch: int = 0,
    pitch_ch: int = 1,
    throttle_ch: int = 2,
    yaw_ch: int = 3,
) -> list[int]:
    """Assemble 16-channel RC array.
    
    If base_channels is provided, it modifies only the dynamically mapped
    roll, pitch, yaw, and throttle axes on top of the base_channels array.
    """
    if base_channels and len(base_channels) >= NUM_CHANNELS:
        ch = list(base_channels[:NUM_CHANNELS])
    else:
        ch = [RC_MID] * NUM_CHANNELS
        ch[4] = 1000  # Default disarm if no base provided

    if 0 <= roll_ch < NUM_CHANNELS:
        ch[roll_ch] = clamp(roll, RC_MIN, RC_MAX)
    if 0 <= pitch_ch < NUM_CHANNELS:
        ch[pitch_ch] = clamp(pitch, RC_MIN, RC_MAX)
    if 0 <= throttle_ch < NUM_CHANNELS:
        ch[throttle_ch] = clamp(throttle, RC_MIN, RC_MAX)
    if 0 <= yaw_ch < NUM_CHANNELS:
        ch[yaw_ch] = clamp(yaw, RC_MIN, RC_MAX)

    return ch
