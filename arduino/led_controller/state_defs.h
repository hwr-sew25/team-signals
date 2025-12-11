#ifndef STATE_DEFS_H
#define STATE_DEFS_H

enum SignalState {
    GREETING,
    IDLE,
    BUSY,
    ERROR_MINOR,
    ERROR_MAJOR,
    LOW_BATTERY,
    MOVE,
    START_MOVE,
    STOP_MOVE,
    REVERSE,
    SPEAKING
};

#endif

