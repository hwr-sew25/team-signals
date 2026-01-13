#!/usr/bin/env python3
"""
REVERSE State - Rückwärtsfahrt.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound


class ReverseState(smach.State):
    """
    REVERSE State - Roboter fährt rückwärts.
    
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
        """Führt die REVERSE-Logik aus."""
        rospy.loginfo("[REVERSE] Entering REVERSE state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf REVERSE setzen
        send_led_command(SignalState.REVERSE)
        
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

