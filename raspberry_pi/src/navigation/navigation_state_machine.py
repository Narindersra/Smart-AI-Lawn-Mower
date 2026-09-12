from enum import Enum, auto


class NavigationState(Enum):
    IDLE = auto()
    DRIVE_LANE = auto()
    APPROACH_BOUNDARY = auto()
    STOP_AT_BOUNDARY = auto()
    TURN_TO_SHIFT = auto()
    SHIFT_LANE = auto()
    STOP_AT_SHIFT = auto()
    TURN_TO_LANE = auto()
    COVERAGE_COMPLETE = auto()
    STOPPED = auto()
    ERROR = auto()

    # Legacy compatibility aliases
    ALIGNING = auto()
    DRIVING = auto()
    TURNING = auto()
    WAYPOINT_REACHED = auto()
    PATH_COMPLETE = auto()


class NavigationStateMachine:

    def __init__(self):
        self.state = NavigationState.IDLE

    def get_state(self):
        return self.state

    def set_state(self, state: NavigationState):
        self.state = state

    def start(self):
        self.state = NavigationState.DRIVE_LANE

    def start_driving(self):
        self.state = NavigationState.DRIVE_LANE

    def start_turning(self):
        self.state = NavigationState.TURN_TO_SHIFT

    def waypoint_reached(self):
        self.state = NavigationState.STOP_AT_BOUNDARY

    def path_complete(self):
        self.state = NavigationState.COVERAGE_COMPLETE

    def stop(self):
        self.state = NavigationState.STOPPED

    def set_error(self):
        self.state = NavigationState.ERROR

    def reset(self):
        self.state = NavigationState.IDLE

    def is_terminal(self):
        return self.state in (
            NavigationState.COVERAGE_COMPLETE,
            NavigationState.PATH_COMPLETE,
            NavigationState.STOPPED,
            NavigationState.ERROR,
        )