#!/usr/bin/env python3
"""
REVERSE State - Rückwärtsfahrt.

HINWEIS: Dieser State ist identisch mit MOVE_BACKWARD und wird aus
Kompatibilitätsgründen beibehalten. Beide verwenden das gleiche LED-Pattern
(Segment 0 / LEDs 0-15 leuchten).
"""

import smach
import rospy

from signal_project.led_engine.led_engine import send_move_direction, DIRECTION_BACKWARD
from signal_project.audio_engine.audio_engine import play_state_sound


class ReverseState(smach.State):
    """
    REVERSE State - Roboter fährt rückwärts.
    
    HINWEIS: Identisch mit MOVE_BACKWARD. Verwendet das gleiche LED-Pattern
    (Segment 0 / LEDs 0-15 leuchten hell, Rest dunkel).
    
    Outcomes:
        - 'done': Rückwärtsfahrt-Signal abgeschlossen, zurück zu IDLE
        - 'preempted': State wurde unterbrochen
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['done', 'preempted'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        """Führt die REVERSE-Logik aus (identisch mit MOVE_BACKWARD)."""
        rospy.loginfo("[REVERSE] Entering REVERSE state (= MOVE_BACKWARD)")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED-Richtung setzen (Segment 0 = Hinten) - identisch mit MOVE_BACKWARD
        send_move_direction(DIRECTION_BACKWARD)
        
        # Sound abspielen
        play_state_sound("reverse.wav")
        
        rospy.loginfo("[REVERSE] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rate.sleep()
        
        return 'done'

