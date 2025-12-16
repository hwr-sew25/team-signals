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


def create_state_machine():
    """
    Erstellt und konfiguriert die SMACH State Machine.
    
    Minimale State Machine: IDLE → GREETING → IDLE
    
    Returns:
        smach.StateMachine: Die konfigurierte State Machine
    """
    # State Machine mit Outcome 'shutdown' erstellen
    sm = smach.StateMachine(outcomes=['shutdown'])
    
    # States zur State Machine hinzufügen
    with sm:
        # IDLE State
        smach.StateMachine.add(
            'IDLE',
            IdleState(),
            transitions={
                'trigger_greeting': 'GREETING',
                'trigger_error': 'IDLE',  # Placeholder für ERROR State
                'trigger_low_battery': 'IDLE',  # Placeholder für LOW_BATTERY State
                'trigger_busy': 'IDLE',  # Placeholder für BUSY State
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

