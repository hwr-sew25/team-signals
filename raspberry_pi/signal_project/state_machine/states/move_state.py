#!/usr/bin/env python3
"""
MOVE State - Roboter bewegt sich in eine Richtung.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import (
    send_move_direction, 
    DIRECTION_FORWARD,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    DIRECTION_BACKWARD
)


class MoveState(smach.State):
    """
    MOVE State - Der Roboter bewegt sich.
    
    Der LED-Strip zeigt die Fahrtrichtung an:
    - Segment 0 (LEDs 0-15):  Links
    - Segment 1 (LEDs 16-31): Vorne
    - Segment 2 (LEDs 32-47): Rechts
    - Segment 3 (LEDs 48-63): Hinten
    
    Die Richtung wird über userdata übergeben oder extern via
    signal_controller_node gesetzt.
    
    Outcomes:
        - 'done': Bewegung angezeigt, zurück zu IDLE
        - 'preempted': State wurde unterbrochen
    """
    
    # Statische Variable für die aktuelle Richtung
    # Kann vom signal_controller_node gesetzt werden
    current_direction = DIRECTION_FORWARD

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['done', 'preempted'],
            input_keys=['direction'],  # Optional: Richtung als Input
            output_keys=[]
        )

    @classmethod
    def set_direction(cls, direction):
        """
        Setzt die aktuelle Fahrtrichtung.
        Kann extern aufgerufen werden (z.B. vom signal_controller_node).
        
        Args:
            direction: DIRECTION_LEFT, DIRECTION_FORWARD, 
                      DIRECTION_RIGHT, DIRECTION_BACKWARD
        """
        cls.current_direction = direction
        rospy.loginfo(f"[MOVE] Direction set to: {direction}")

    def execute(self, userdata):
        """Führt die MOVE-Logik aus."""
        rospy.loginfo("[MOVE] Entering MOVE state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # Richtung aus userdata nehmen, falls vorhanden
        # In SMACH funktioniert hasattr() nicht - daher try/except
        direction = self.current_direction
        try:
            if userdata.direction is not None:
                direction = userdata.direction
        except (KeyError, AttributeError):
            # Kein direction in userdata - nutze current_direction
            pass
        
        # LED-Richtung setzen
        send_move_direction(direction)
        
        rospy.loginfo(f"[MOVE] Moving with direction: {direction}")
        return 'done'

