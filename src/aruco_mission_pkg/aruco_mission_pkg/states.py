from enum import Enum, auto

class MissionState(Enum):
    TAKEOFF = auto()
    SEARCH = auto()
    ALIGN = auto()
    APPROACH = auto()
    PASS = auto()
    IDLE = auto()