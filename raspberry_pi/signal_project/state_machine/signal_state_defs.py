from enum import Enum

class SignalState(Enum):
    GREETING = 0
    IDLE = 1
    BUSY = 2
    STOP_BUSY = 3
    ERROR_MINOR_STUCK = 4
    ERROR_MINOR_NAV = 5
    ROOM_NOT_FOUND = 6
    ERROR_MAJOR = 7
    LOW_BATTERY = 8
    MOVE = 9
    START_MOVE = 10
    STOP_MOVE = 11
    GOAL_REACHED = 12
    REVERSE = 13
    SPEAKING = 14

