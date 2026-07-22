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
        self._last_log_time = 0.0

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
            
        # Send RC Override
        # Channel map (AETR): 1=Roll, 2=Pitch, 3=Throttle, 4=Yaw
        # AUX1 (Ch 5) = Arm switch (2000 = Arm, 1000 = Disarm)
        # AUX2 (Ch 6) = Flight Mode switch (1800..2000 = Angle Mode, <1800 = Acro Mode)
        ch5 = 2000 if self._armed else 1000
        mode_str = getattr(self, '_flight_mode', 'ANGLE').upper()
        ch6 = 1900 if mode_str == "ANGLE" else 1000
        
        # 65535 prevents RC Failsafe on unused channels
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component,
            roll, pitch, throttle, yaw, ch5, ch6, 65535, 65535
        )
        
        now = time.time()
        if now - self._last_log_time >= 1.0:
            self.sys_log.log(
                LogCategory.DRONE,
                f"MAVLink RC sent: R{roll} P{pitch} Y{yaw} T{throttle} | AUX1(Arm):{ch5} AUX2(Mode {mode_str}):{ch6}",
                module="MAVLink"
            )
            self._last_log_time = now

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
