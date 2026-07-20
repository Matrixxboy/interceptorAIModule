from abc import ABC, abstractmethod
from typing import Dict, Any

class FlightController(ABC):
    """
    Abstract base class for flight controllers (e.g., MSP, MAVLink).
    """

    @abstractmethod
    def connect(self) -> bool:
        """Connect to the flight controller."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the flight controller."""
        pass
        
    @abstractmethod
    def is_connected(self) -> bool:
        """Check if connected."""
        pass

    @abstractmethod
    def arm(self) -> bool:
        """Arm the drone."""
        pass

    @abstractmethod
    def disarm(self) -> bool:
        """Disarm the drone."""
        pass

    @abstractmethod
    def send_control(self, roll: int, pitch: int, yaw: int, throttle: int) -> None:
        """Send RC control inputs."""
        pass

    @abstractmethod
    def get_telemetry(self) -> Dict[str, Any]:
        """Get the latest telemetry data."""
        pass
