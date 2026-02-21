from enum import Enum

class MissionState(Enum):
    TAKEOFF = 0
    SEARCH = 1
    ALIGN = 2
    APPROACH = 3
    PASS = 4
    DONE = 5