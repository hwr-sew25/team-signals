#!/usr/bin/env python3
"""
LOW_BATTERY State - Niedriger Akkustand.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound


class LowBatteryState(smach.State):
    """
    LOW_BATTERY State - Akkustand niedrig.
    
    Outcomes:
        - 'done': Warnung angezeigt, zurück zu IDLE
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
        """Führt die LOW_BATTERY-Logik aus."""
        rospy.loginfo("[LOW_BATTERY] Entering LOW_BATTERY state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf LOW_BATTERY setzen
        send_led_command(SignalState.LOW_BATTERY)
        
        # Sound abspielen
        play_state_sound("lowbattery.wav")
        
        rospy.loginfo("[LOW_BATTERY] Low battery warning complete, returning to IDLE")
        return 'done'

