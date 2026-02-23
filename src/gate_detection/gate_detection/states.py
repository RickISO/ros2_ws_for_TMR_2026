from enum import Enum, auto

class MissionState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    SEARCH_GATE = auto()
    ALIGN_GATE = auto()
    APPROACH_GATE = auto()
    PASS_GATE = auto()
    FINISHED = auto()