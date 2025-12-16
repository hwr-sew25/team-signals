#!/usr/bin/env python3
"""
Legacy-Modul - wird für Rückwärtskompatibilität beibehalten.
Bitte verwende signal_project.state_machine.state_machine stattdessen.
"""

# Re-export für Rückwärtskompatibilität
from signal_project.state_machine.state_machine import create_state_machine, get_idle_state

# Legacy trigger_state Funktion (deprecated)
def trigger_state(state):
    """
    DEPRECATED: Diese Funktion ist nicht mehr verfügbar.
    Verwende stattdessen die SMACH State Machine.
    """
    raise NotImplementedError(
        "trigger_state() ist deprecated. "
        "Verwende die SMACH State Machine mit create_state_machine() stattdessen."
    )
