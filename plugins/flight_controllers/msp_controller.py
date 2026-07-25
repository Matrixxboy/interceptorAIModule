"""MSP flight controller plugin for Betaflight / INAV."""

from __future__ import annotations

import logging
from typing import Any

import serial

from control import msp_link
from interfaces.flight_controller import FlightController


class MSPController(FlightController):
    """Flight controller plugin for Betaflight/INAV using MSP."""

    def __init__(
        self,
        port: str = "COM4",
        baudrate: int = 115200,
        channel_map: str = "AETR",
        arm_channel: int = 4,
        mode_channel: int = 5,
        arm_high: int = 1800,
        arm_low: int = 1000,
        mode_high: int = 1900,
        mode_low: int = 1000,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.channel_map = channel_map
        self.arm_channel = arm_channel
        self.mode_channel = mode_channel
        self.arm_high = arm_high
        self.arm_low = arm_low
        self.mode_high = mode_high
        self.mode_low = mode_low
        self.ser = None
        self.logger = logging.getLogger("MSPController")
        self._armed = False
        self._flight_mode = "ANGLE"
        self._channel_overrides: dict[int, int] = {}

    def update_aux_config(
        self,
        arm_channel: int,
        mode_channel: int,
        arm_high: int,
        arm_low: int,
        mode_high: int,
        mode_low: int,
    ) -> None:
        self.arm_channel = arm_channel
        self.mode_channel = mode_channel
        self.arm_high = arm_high
        self.arm_low = arm_low
        self.mode_high = mode_high
        self.mode_low = mode_low

    def set_channel_overrides(self, overrides: dict[int, int] | None) -> None:
        self._channel_overrides = dict(overrides or {})

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.logger.info(f"Connected to MSP on {self.port} at {self.baudrate} baud.")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to MSP on {self.port}: {e}")
            self.ser = None
            return False

    def disconnect(self) -> None:
        if self.ser and self.ser.is_open:
            self.disarm()
            self.ser.close()
            self.logger.info("Disconnected from MSP.")
        self.ser = None

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _build_channels(
        self,
        roll: int,
        pitch: int,
        yaw: int,
        throttle: int,
        arm: bool,
        flight_mode: bool,
    ) -> list[int]:
        return msp_link.make_rc_channels(
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            throttle=throttle,
            arm=arm,
            flight_mode=flight_mode,
            channel_map=self.channel_map,
            arm_channel=self.arm_channel,
            mode_channel=self.mode_channel,
            arm_high=self.arm_high,
            arm_low=self.arm_low,
            mode_high=self.mode_high,
            mode_low=self.mode_low,
            channel_overrides=self._channel_overrides,
        )

    def arm(self) -> bool:
        if not self.is_connected():
            return False
        self.logger.warning("Attempting to arm drone via MSP.")
        channels = self._build_channels(
            msp_link.RC_MID, msp_link.RC_MID, msp_link.RC_MID, msp_link.RC_MIN,
            arm=True, flight_mode=True,
        )
        try:
            self.ser.write(msp_link.build_msp_set_raw_rc(channels))
            self._armed = True
            return True
        except Exception as e:
            self.logger.error(f"Failed to send arm command: {e}")
            return False

    def disarm(self) -> bool:
        if not self.is_connected():
            return False
        self.logger.info("Disarming drone.")
        channels = self._build_channels(
            msp_link.RC_MID, msp_link.RC_MID, msp_link.RC_MID, msp_link.RC_MIN,
            arm=False, flight_mode=False,
        )
        try:
            self.ser.write(msp_link.build_msp_set_raw_rc(channels))
            self._armed = False
            return True
        except Exception as e:
            self.logger.error(f"Failed to send disarm command: {e}")
            return False

    def set_flight_mode(self, mode: str) -> None:
        self._flight_mode = mode.upper()

    def send_control(self, roll: int, pitch: int, yaw: int, throttle: int) -> None:
        if not self.is_connected():
            return
        is_angle = getattr(self, "_flight_mode", "ANGLE").upper() == "ANGLE"
        channels = self._build_channels(
            roll, pitch, yaw, throttle,
            arm=self._armed,
            flight_mode=is_angle or self._armed,
        )
        try:
            self.ser.write(msp_link.build_msp_set_raw_rc(channels))
        except Exception as e:
            self.logger.error(f"Failed to send control command: {e}")

    def get_attitude(self) -> dict[str, float]:
        """Single cheap MSP_ATTITUDE round-trip — used for camera levelling."""
        if not self.is_connected():
            return {}
        try:
            self.ser.write(msp_link.build_msp_request(msp_link.MSP_ATTITUDE))
            resp = msp_link.read_msp_response(self.ser)
            if resp and resp[0] == msp_link.MSP_ATTITUDE:
                return msp_link.parse_msp_attitude(resp[1]) or {}
        except Exception as e:
            self.logger.error(f"Error reading attitude: {e}")
        return {}

    def get_telemetry(self) -> dict[str, Any]:
        telemetry: dict[str, Any] = {}
        if not self.is_connected():
            return telemetry
        try:
            att = self.get_attitude()
            if att:
                telemetry.update(att)

            self.ser.write(msp_link.build_msp_request(msp_link.MSP_STATUS_EX))
            resp = msp_link.read_msp_response(self.ser)
            if resp and resp[0] == msp_link.MSP_STATUS_EX:
                status_data = msp_link.parse_msp_status_ex(resp[1])
                if status_data:
                    telemetry.update(status_data)
                    self._armed = status_data.get("armed", False)

            self.ser.write(msp_link.build_msp_request(msp_link.MSP_ALTITUDE))
            resp = msp_link.read_msp_response(self.ser)
            if resp and resp[0] == msp_link.MSP_ALTITUDE:
                alt_data = msp_link.parse_msp_altitude(resp[1])
                if alt_data:
                    telemetry.update(alt_data)

            self.ser.write(msp_link.build_msp_request(msp_link.MSP_ANALOG))
            resp = msp_link.read_msp_response(self.ser)
            if resp and resp[0] == msp_link.MSP_ANALOG:
                analog_data = msp_link.parse_msp_analog(resp[1])
                if analog_data:
                    telemetry.update(analog_data)
        except Exception as e:
            self.logger.error(f"Error reading telemetry: {e}")
        return telemetry
