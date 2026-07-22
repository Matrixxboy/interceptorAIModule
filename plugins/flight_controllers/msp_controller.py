import serial
import time
import logging
from typing import Dict, Any
from interfaces.flight_controller import FlightController
from control import msp_link

class MSPController(FlightController):
    """
    Flight controller plugin for Betaflight/INAV using the MSP protocol.
    """
    
    def __init__(self, port: str = "COM4", baudrate: int = 115200, channel_map: str = "AETR"):
        self.port = port
        self.baudrate = baudrate
        self.channel_map = channel_map
        self.ser = None
        self.logger = logging.getLogger("MSPController")
        self._armed = False

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
            self.disarm() # Try to disarm before closing
            self.ser.close()
            self.logger.info("Disconnected from MSP.")
        self.ser = None

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def arm(self) -> bool:
        if not self.is_connected():
            return False
        
        self.logger.warning("Attempting to arm drone via MSP.")
        # Send arm command
        channels = msp_link.make_rc_channels(
            roll=msp_link.RC_MID, pitch=msp_link.RC_MID, 
            yaw=msp_link.RC_MID, throttle=msp_link.RC_MIN, 
            arm=True, channel_map=self.channel_map
        )
        payload = msp_link.build_msp_set_raw_rc(channels)
        try:
            self.ser.write(payload)
            self._armed = True
            return True
        except Exception as e:
            self.logger.error(f"Failed to send arm command: {e}")
            return False

    def disarm(self) -> bool:
        if not self.is_connected():
            return False
            
        self.logger.info("Disarming drone.")
        channels = msp_link.make_rc_channels(
            roll=msp_link.RC_MID, pitch=msp_link.RC_MID, 
            yaw=msp_link.RC_MID, throttle=msp_link.RC_MIN, 
            arm=False, channel_map=self.channel_map
        )
        payload = msp_link.build_msp_set_raw_rc(channels)
        try:
            self.ser.write(payload)
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
            
        is_angle = getattr(self, '_flight_mode', 'ANGLE').upper() == "ANGLE"
        channels = msp_link.make_rc_channels(
            roll=roll, pitch=pitch, yaw=yaw, throttle=throttle,
            arm=self._armed, flight_mode=is_angle, channel_map=self.channel_map
        )
        payload = msp_link.build_msp_set_raw_rc(channels)
        try:
            self.ser.write(payload)
        except Exception as e:
            self.logger.error(f"Failed to send control command: {e}")

    def get_telemetry(self) -> Dict[str, Any]:
        telemetry = {}
        if not self.is_connected():
            return telemetry
            
        # Request status
        try:
            self.ser.write(msp_link.build_msp_request(msp_link.MSP_STATUS_EX))
            resp = msp_link.read_msp_response(self.ser)
            if resp and resp[0] == msp_link.MSP_STATUS_EX:
                status_data = msp_link.parse_msp_status_ex(resp[1])
                if status_data:
                    telemetry.update(status_data)
                    self._armed = status_data.get("armed", False)
                    
            # Request altitude
            self.ser.write(msp_link.build_msp_request(msp_link.MSP_ALTITUDE))
            resp = msp_link.read_msp_response(self.ser)
            if resp and resp[0] == msp_link.MSP_ALTITUDE:
                alt_data = msp_link.parse_msp_altitude(resp[1])
                if alt_data:
                    telemetry.update(alt_data)
                    
            # Request analog
            self.ser.write(msp_link.build_msp_request(msp_link.MSP_ANALOG))
            resp = msp_link.read_msp_response(self.ser)
            if resp and resp[0] == msp_link.MSP_ANALOG:
                analog_data = msp_link.parse_msp_analog(resp[1])
                if analog_data:
                    telemetry.update(analog_data)
                    
        except Exception as e:
            self.logger.error(f"Error reading telemetry: {e}")
            
        return telemetry
