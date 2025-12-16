#!/usr/bin/env python3
"""
SMACH States für die Signal State Machine.
"""

from signal_project.state_machine.states.idle_state import IdleState
from signal_project.state_machine.states.greeting_state import GreetingState
from signal_project.state_machine.states.error_minor_state import ErrorMinorState
from signal_project.state_machine.states.error_major_state import ErrorMajorState
from signal_project.state_machine.states.low_battery_state import LowBatteryState
from signal_project.state_machine.states.start_move_state import StartMoveState
from signal_project.state_machine.states.stop_move_state import StopMoveState
from signal_project.state_machine.states.reverse_state import ReverseState
from signal_project.state_machine.states.speaking_state import SpeakingState
