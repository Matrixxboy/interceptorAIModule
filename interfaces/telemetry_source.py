from abc import ABC, abstractmethod
from typing import Dict, Any

class TelemetrySource(ABC):
    """
    Abstract base class for sources of telemetry (e.g., GPS, external sensors).
    """

    @abstractmethod
    def connect(self) -> bool:
        """Connect to the telemetry source."""
        pass

    @abstractmethod
    def read_data(self) -> Dict[str, Any]:
        """Read data from the source."""
        pass
        
    @abstractmethod
    def close(self) -> None:
        """Close connection."""
        pass
