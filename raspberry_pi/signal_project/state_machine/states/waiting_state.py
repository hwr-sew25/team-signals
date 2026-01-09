#!/usr/bin/env python3
"""
WAITING State - Der Roboter wartet auf Bestätigung/Countdown.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound


class WaitingState(smach.State):
    """
    WAITING State - Der Roboter wartet auf Bestätigung.
    
    Wird verwendet während:
    - Display zeigt Countdown zur Bestätigung
    - System verarbeitet die Eingabe
    - Warten auf externe Bestätigung
    
    LED-Animation: Lichtwelle die durch den Strip läuft (wie Ladekreis)
    
    Outcomes:
        - 'done': Warten beendet, zurück zu IDLE
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
        """Führt die WAITING-Logik aus."""
        rospy.loginfo("[WAITING] Entering WAITING state - showing loading animation")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf WAITING setzen (Lichtwellen-Animation)
        send_led_command(SignalState.WAITING)
        
        # Optional: Sound abspielen
        # play_state_sound("waiting.wav")
        
        rospy.loginfo("[WAITING] Waiting state activated - LED wave animation running")
        return 'done'

