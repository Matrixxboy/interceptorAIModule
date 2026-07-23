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
    arm: bool = False,
    flight_mode: bool = False,
    aux1: int | None = None,
    aux2: int | None = None,
    aux3: int | None = None,
    aux4: int | None = None,
    channel_map: str = "AETR",
    arm_channel: int = ARM_CH,
    mode_channel: int = MODE_CH,
    arm_high: int = ARM_HIGH,
    arm_low: int = ARM_LOW,
    mode_high: int = MODE_HIGH,
    mode_low: int = MODE_LOW,
    channel_overrides: dict[int, int] | None = None,
) -> list[int]:
    """Assemble 16-channel RC array supporting AETR/TAER maps and AUX overrides.

    Channel indices are 0-based (CH1=0 … CH16=15).
    AUX1 is typically index 4 (CH5), AUX2 index 5 (CH6).
    """
    ch = [RC_MID] * NUM_CHANNELS

    map_str = channel_map.upper()
    if map_str.startswith("TAER"):
        ch[0] = clamp(throttle, RC_MIN, RC_MAX)
        ch[1] = clamp(roll, RC_MIN, RC_MAX)
        ch[2] = clamp(pitch, RC_MIN, RC_MAX)
        ch[3] = clamp(yaw, RC_MIN, RC_MAX)
    else:  # AETR
        ch[0] = clamp(roll, RC_MIN, RC_MAX)
        ch[1] = clamp(pitch, RC_MIN, RC_MAX)
        ch[2] = clamp(throttle, RC_MIN, RC_MAX)
        ch[3] = clamp(yaw, RC_MIN, RC_MAX)

    # Default AUX1–AUX4 mid/low unless explicitly provided
    ch[4] = clamp(aux1 if aux1 is not None else RC_MID, RC_MIN, RC_MAX)
    ch[5] = clamp(aux2 if aux2 is not None else RC_MID, RC_MIN, RC_MAX)
    ch[6] = clamp(aux3 if aux3 is not None else RC_MID, RC_MIN, RC_MAX)
    ch[7] = clamp(aux4 if aux4 is not None else RC_MID, RC_MIN, RC_MAX)

    # Apply configured ARM / Mode channels (may be same as AUX1/AUX2)
    a_ch = max(0, min(NUM_CHANNELS - 1, int(arm_channel)))
    m_ch = max(0, min(NUM_CHANNELS - 1, int(mode_channel)))
    ch[a_ch] = clamp(arm_high if arm else arm_low, RC_MIN, RC_MAX)
    ch[m_ch] = clamp(mode_high if flight_mode else mode_low, RC_MIN, RC_MAX)

    # Explicit per-channel overrides (joystick AUX mapping, etc.)
    if channel_overrides:
        for idx, pwm in channel_overrides.items():
            i = int(idx)
            if 0 <= i < NUM_CHANNELS:
                ch[i] = clamp(pwm, RC_MIN, RC_MAX)

    return ch
