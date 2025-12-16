#!/usr/bin/env python3
"""
GREETING State - Begrüßungszustand des Roboters.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound


class GreetingState(smach.State):
    """
    GREETING State - Der Roboter begrüßt den Benutzer.
    
    Outcomes:
        - 'done': Begrüßung abgeschlossen, zurück zu IDLE
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
        """Führt die GREETING-Logik aus."""
        rospy.loginfo("[GREETING] Entering GREETING state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf GREETING setzen
        send_led_command(SignalState.GREETING)
        
        # Sound abspielen
        play_state_sound("greeting.wav")
        
        rospy.loginfo("[GREETING] Greeting complete, returning to IDLE")
        return 'done'

