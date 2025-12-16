#!/usr/bin/env python3
"""
Zentrale SMACH State Machine für das Signal Project.

Diese Datei definiert alle State-Transitions.
Audio- und LED-Logik wird nur innerhalb der States aufgerufen.
"""

import smach
import rospy

from signal_project.state_machine.states.idle_state import IdleState
from signal_project.state_machine.states.greeting_state import GreetingState
from signal_project.state_machine.states.error_minor_state import ErrorMinorState
from signal_project.state_machine.states.error_major_state import ErrorMajorState
from signal_project.state_machine.states.low_battery_state import LowBatteryState
from signal_project.state_machine.states.start_move_state import StartMoveState
from signal_project.state_machine.states.stop_move_state import StopMoveState
from signal_project.state_machine.states.reverse_state import ReverseState
from signal_project.state_machine.states.speaking_state import SpeakingState


def create_state_machine():
    """
    Erstellt und konfiguriert die SMACH State Machine.
    
    State Machine mit allen States:
    IDLE → (trigger) → STATE → IDLE
    
    Returns:
        smach.StateMachine: Die konfigurierte State Machine
    """
    # State Machine mit Outcome 'shutdown' erstellen
    sm = smach.StateMachine(outcomes=['shutdown'])
    
    # States zur State Machine hinzufügen
    with sm:
        # IDLE State - Zentrale Warteschleife
        smach.StateMachine.add(
            'IDLE',
            IdleState(),
            transitions={
                'trigger_greeting': 'GREETING',
                'trigger_error_minor': 'ERROR_MINOR',
                'trigger_error_major': 'ERROR_MAJOR',
                'trigger_low_battery': 'LOW_BATTERY',
                'trigger_start_move': 'START_MOVE',
                'trigger_stop_move': 'STOP_MOVE',
                'trigger_reverse': 'REVERSE',
                'trigger_speaking': 'SPEAKING',
                'preempted': 'shutdown'
            }
        )
        
        # GREETING State
        smach.StateMachine.add(
            'GREETING',
            GreetingState(),
            transitions={
                'done': 'IDLE',
                'preempted': 'shutdown'
            }
        )
        
        # ERROR_MINOR State
        smach.StateMachine.add(
            'ERROR_MINOR',
            ErrorMinorState(),
            transitions={
                'done': 'IDLE',
                'preempted': 'shutdown'
            }
        )
        
        # ERROR_MAJOR State
        smach.StateMachine.add(
            'ERROR_MAJOR',
            ErrorMajorState(),
            transitions={
                'done': 'IDLE',
                'preempted': 'shutdown'
            }
        )
        
        # LOW_BATTERY State
        smach.StateMachine.add(
            'LOW_BATTERY',
            LowBatteryState(),
            transitions={
                'done': 'IDLE',
                'preempted': 'shutdown'
            }
        )
        
        # START_MOVE State
        smach.StateMachine.add(
            'START_MOVE',
            StartMoveState(),
            transitions={
                'done': 'IDLE',
                'preempted': 'shutdown'
            }
        )
        
        # STOP_MOVE State
        smach.StateMachine.add(
            'STOP_MOVE',
            StopMoveState(),
            transitions={
                'done': 'IDLE',
                'preempted': 'shutdown'
            }
        )
        
        # REVERSE State
        smach.StateMachine.add(
            'REVERSE',
            ReverseState(),
            transitions={
                'done': 'IDLE',
                'preempted': 'shutdown'
            }
        )
        
        # SPEAKING State
        smach.StateMachine.add(
            'SPEAKING',
            SpeakingState(),
            transitions={
                'done': 'IDLE',
                'preempted': 'shutdown'
            }
        )
    
    return sm


def get_idle_state(sm):
    """
    Holt den IDLE State aus der State Machine für externe Trigger.
    
    Args:
        sm: Die SMACH State Machine
        
    Returns:
        IdleState: Der IDLE State oder None
    """
    if 'IDLE' in sm._states:
        return sm._states['IDLE']
    return None
