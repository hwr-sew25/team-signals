import sys
import os
BASE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)
sys.path.append(BASE_PATH)

from state_machine.signal_state_machine import trigger_state
from state_machine.signal_state_defs import SignalState

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

