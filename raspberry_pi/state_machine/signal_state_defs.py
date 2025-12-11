from enum import Enum

class SignalState(Enum):
    GREETING = 0
    IDLE = 1
    BUSY = 2
    ERROR_MINOR = 3
    ERROR_MAJOR = 4
    LOW_BATTERY = 5
    MOVE = 6
    START_MOVE = 7
    STOP_MOVE = 8
    REVERSE = 9
    SPEAKING = 10

