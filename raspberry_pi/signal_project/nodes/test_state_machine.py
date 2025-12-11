#!/usr/bin/env python3
import sys
import os

from signal_project.state_machine.signal_state_machine import trigger_state
from signal_project.state_machine.signal_state_defs import SignalState

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def main():
    print("=== TEAM SIGNALE — COMPLETE STATE MACHINE TEST ===")
    print("Verfügbare States:")
    for s in SignalState:
        print(f" - {s.name}")

    while True:
        user_input = input("\nState > ").strip().upper()

        if user_input in SignalState.__members__:
            state = SignalState[user_input]
            trigger_state(state)
        else:
            print("Unbekannter State.")

if __name__ == "__main__":
    main()

