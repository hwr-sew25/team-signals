#!/usr/bin/env python3
"""
ERROR_MINOR State - Leichter Fehlerzustand.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound


class ErrorMinorState(smach.State):
    """
    ERROR_MINOR State - Leichter Fehler aufgetreten.
    
    Outcomes:
        - 'done': Fehler angezeigt, zurück zu IDLE
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
        """Führt die ERROR_MINOR-Logik aus."""
        rospy.loginfo("[ERROR_MINOR] Entering ERROR_MINOR state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf ERROR_MINOR setzen
        send_led_command(SignalState.ERROR_MINOR)
        
        # Sound abspielen
        play_state_sound("error_minor.wav")
        
        rospy.loginfo("[ERROR_MINOR] Error minor complete, returning to IDLE")
        return 'done'

