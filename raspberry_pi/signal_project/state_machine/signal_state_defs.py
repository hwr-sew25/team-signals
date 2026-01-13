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
    MOVE_LEFT = 9
    MOVE_FORWARD = 10
    MOVE_RIGHT = 11
    MOVE_BACKWARD = 12
    START_MOVE = 13
    STOP_MOVE = 14
    GOAL_REACHED = 15
    REVERSE = 16
    SPEAKING = 17
    WAITING = 18

