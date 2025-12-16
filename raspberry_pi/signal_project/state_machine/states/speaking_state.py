#!/usr/bin/env python3
"""
SPEAKING State - Roboter spricht.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound


class SpeakingState(smach.State):
    """
    SPEAKING State - Roboter gibt Sprachausgabe.
    
    Outcomes:
        - 'done': Sprachausgabe abgeschlossen, zurück zu IDLE
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
        """Führt die SPEAKING-Logik aus."""
        rospy.loginfo("[SPEAKING] Entering SPEAKING state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf SPEAKING setzen
        send_led_command(SignalState.SPEAKING)
        
        # Sound abspielen
        play_state_sound("speaking.wav")
        
        rospy.loginfo("[SPEAKING] Speaking complete, returning to IDLE")
        return 'done'

