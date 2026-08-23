"""State Machine for Target Tracking and Following."""

from enum import Enum, auto


class TrackingState(Enum):
    IDLE = auto()
    TARGET_LOCKED = auto()
    FOLLOWING = auto()


class TargetTrackingStateMachine:
    def __init__(self) -> None:
        self.state = TrackingState.IDLE

    def update(self, lock_switch: bool, follow_switch: bool, has_valid_target: bool) -> TrackingState:
        """
        Updates the state machine based on RC switch inputs and tracking status.
        Ensures safe fallbacks when a target is lost or switches are toggled.
        """
        if self.state == TrackingState.IDLE:
            if lock_switch:
                self.state = TrackingState.TARGET_LOCKED

        elif self.state == TrackingState.TARGET_LOCKED:
            if not lock_switch:
                self.state = TrackingState.IDLE
            elif follow_switch and has_valid_target:
                self.state = TrackingState.FOLLOWING

        elif self.state == TrackingState.FOLLOWING:
            if not lock_switch:
                self.state = TrackingState.IDLE
            elif not follow_switch or not has_valid_target:
                self.state = TrackingState.TARGET_LOCKED

        return self.state
