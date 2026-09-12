from enum import Enum, auto


class NavigationState(Enum):
    IDLE = auto()
    ALIGNING = auto()
    DRIVING = auto()
    TURNING = auto()
    WAYPOINT_REACHED = auto()
    PATH_COMPLETE = auto()
    STOPPED = auto()
    ERROR = auto()


class NavigationStateMachine:

    def __init__(self):
        self.state = NavigationState.IDLE

    def get_state(self):
        return self.state

    def start(self):
        if self.state in (
            NavigationState.IDLE,
            NavigationState.STOPPED,
            NavigationState.WAYPOINT_REACHED,
            NavigationState.TURNING,
        ):
            self.state = NavigationState.ALIGNING

    def start_driving(self):
        if self.state in (
            NavigationState.ALIGNING,
            NavigationState.TURNING,
        ):
            self.state = NavigationState.DRIVING

    def start_turning(self):
        if self.state in (
            NavigationState.ALIGNING,
            NavigationState.DRIVING,
            NavigationState.WAYPOINT_REACHED,
        ):
            self.state = NavigationState.TURNING

    def waypoint_reached(self):
        if self.state in (
            NavigationState.ALIGNING,
            NavigationState.DRIVING,
            NavigationState.TURNING,
        ):
            self.state = NavigationState.WAYPOINT_REACHED

    def path_complete(self):
        self.state = NavigationState.PATH_COMPLETE

    def stop(self):
        self.state = NavigationState.STOPPED

    def set_error(self):
        self.state = NavigationState.ERROR

    def reset(self):
        self.state = NavigationState.IDLE

    def is_terminal(self):
        return self.state in (
            NavigationState.PATH_COMPLETE,
            NavigationState.STOPPED,
            NavigationState.ERROR,
        )