import logging
import time
from typing import Dict, Any
from interfaces.flight_controller import FlightController
from sys_logging.system_logger import SystemLogger, LogCategory

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

class MAVLinkController(FlightController):
    """
    Flight controller plugin for ArduPilot/PX4 using MAVLink.
    """
    def __init__(self, connection_string: str = "udp:127.0.0.1:14550", baudrate: int = 57600):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None
        self.logger = logging.getLogger("MAVLinkController")
        self.sys_log = SystemLogger()
        self._armed = False
        self._flight_mode = "ANGLE"
        self._last_log_time = 0.0
        self._channel_overrides: dict[int, int] = {}
        self.arm_channel = 4
        self.mode_channel = 5
        self.arm_high = 1800
        self.arm_low = 1000
        self.mode_high = 1900
        self.mode_low = 1000

    def set_channel_overrides(self, overrides: dict[int, int] | None) -> None:
        self._channel_overrides = dict(overrides or {})

    def update_aux_config(
        self,
        arm_channel: int = 4,
        mode_channel: int = 5,
        arm_high: int = 1800,
        arm_low: int = 1000,
        mode_high: int = 1900,
        mode_low: int = 1000,
    ) -> None:
        self.arm_channel = int(arm_channel)
        self.mode_channel = int(mode_channel)
        self.arm_high = int(arm_high)
        self.arm_low = int(arm_low)
        self.mode_high = int(mode_high)
        self.mode_low = int(mode_low)

    def connect(self) -> bool:
        if mavutil is None:
            self.logger.error("pymavlink is not installed. Cannot use MAVLinkController.")
            return False
            
        try:
            self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baudrate)
            hb = self.master.wait_heartbeat(timeout=3)
            if hb is not None:
                self.logger.info(f"Connected to MAVLink vehicle at {self.connection_string} (Sys {self.master.target_system}, Comp {self.master.target_component})")
            else:
                self.logger.warning(f"No initial MAVLink heartbeat on {self.connection_string}, port opened.")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to MAVLink vehicle: {e}")
            self.master = None
            return False

    def disconnect(self) -> None:
        if self.master:
            self.master.close()
            self.master = None
            self.logger.info("Disconnected from MAVLink vehicle.")

    def is_connected(self) -> bool:
        return self.master is not None

    def arm(self) -> bool:
        if not self.is_connected():
            return False
            
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        self.sys_log.log(LogCategory.DRONE, "Sent MAVLink ARM command (MAV_CMD)", module="MAVLink")
        self._armed = True
        return True

    def disarm(self) -> bool:
        if not self.is_connected():
            return False
            
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        self.sys_log.log(LogCategory.DRONE, "Sent MAVLink DISARM command (MAV_CMD)", module="MAVLink")
        self._armed = False
        return True

    def set_flight_mode(self, mode: str) -> None:
        self._flight_mode = mode.upper()

    def send_control(self, roll: int, pitch: int, yaw: int, throttle: int) -> None:
        if not self.is_connected():
            return

        # Build 8-channel RC: AETR + AUX arm/mode (+ overrides)
        ch = [65535] * 8
        ch[0], ch[1], ch[2], ch[3] = int(roll), int(pitch), int(throttle), int(yaw)

        mode_str = getattr(self, "_flight_mode", "ANGLE").upper()
        arm_idx = max(0, min(7, int(getattr(self, "arm_channel", 4))))
        mode_idx = max(0, min(7, int(getattr(self, "mode_channel", 5))))
        ch[arm_idx] = int(self.arm_high if self._armed else self.arm_low)
        ch[mode_idx] = int(self.mode_high if mode_str == "ANGLE" else self.mode_low)

        for idx, pwm in getattr(self, "_channel_overrides", {}).items():
            i = int(idx)
            if 0 <= i < 8:
                ch[i] = int(pwm)
        # Re-assert arm/mode after overrides
        ch[arm_idx] = int(self.arm_high if self._armed else self.arm_low)
        ch[mode_idx] = int(self.mode_high if mode_str == "ANGLE" else self.mode_low)

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7],
        )

        now = time.time()
        if now - self._last_log_time >= 1.0:
            self.sys_log.log(
                LogCategory.DRONE,
                f"MAVLink RC: R{ch[0]} P{ch[1]} T{ch[2]} Y{ch[3]} | "
                f"ARM(ch{arm_idx+1}):{ch[arm_idx]} MODE:{ch[mode_idx]}",
                module="MAVLink",
            )
            self._last_log_time = now

    def get_attitude(self) -> Dict[str, float]:
        """Latest roll/pitch/yaw in degrees — used for camera levelling."""
        tel = self.get_telemetry()
        att = {k: tel[k] for k in ("roll_deg", "pitch_deg", "yaw_deg") if k in tel}
        if att:
            self._last_attitude = att
        return att or getattr(self, "_last_attitude", {})

    def get_telemetry(self) -> Dict[str, Any]:
        telemetry = {}
        if not self.is_connected():
            return telemetry
            
        try:
            # Drain all pending messages so we don't lag behind
            while True:
                msg = self.master.recv_match(type=['ATTITUDE', 'VFR_HUD', 'HEARTBEAT'], blocking=False)
                if not msg:
                    break
                
                msg_type = msg.get_type()
                if msg_type == 'ATTITUDE':
                    telemetry['roll_deg'] = msg.roll * 57.2958
                    telemetry['pitch_deg'] = msg.pitch * 57.2958
                    telemetry['yaw_deg'] = msg.yaw * 57.2958
                elif msg_type == 'VFR_HUD':
                    telemetry['alt_m'] = msg.alt
                elif msg_type == 'HEARTBEAT':
                    self._armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    telemetry['armed'] = self._armed
        except Exception as e:
            self.logger.error(f"Error reading MAVLink telemetry: {e}")
            
        return telemetry
