import logging
import time
from typing import Dict, Any
from interfaces.flight_controller import FlightController

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
        self._armed = False

    def connect(self) -> bool:
        if mavutil is None:
            self.logger.error("pymavlink is not installed. Cannot use MAVLinkController.")
            return False
            
        try:
            self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baudrate)
            self.master.wait_heartbeat(timeout=5)
            self.logger.info(f"Connected to MAVLink vehicle at {self.connection_string}")
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
        self.logger.info("Sent MAVLink ARM command.")
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
        self.logger.info("Sent MAVLink DISARM command.")
        self._armed = False
        return True

    def send_control(self, roll: int, pitch: int, yaw: int, throttle: int) -> None:
        if not self.is_connected():
            return
            
        # Send RC Override
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component,
            roll, pitch, throttle, yaw, 0, 0, 0, 0
        )

    def get_telemetry(self) -> Dict[str, Any]:
        telemetry = {}
        if not self.is_connected():
            return telemetry
            
        try:
            msg = self.master.recv_match(type=['ATTITUDE', 'VFR_HUD', 'HEARTBEAT'], blocking=False)
            if msg:
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
