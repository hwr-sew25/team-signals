#!/usr/bin/env python3
"""
START_MOVE State - Bewegung beginnt.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound


class StartMoveState(smach.State):
    """
    START_MOVE State - Roboter beginnt Bewegung.
    
    Outcomes:
        - 'done': Bewegungsstart signalisiert, zurück zu IDLE
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
        """Führt die START_MOVE-Logik aus."""
        rospy.loginfo("[START_MOVE] Entering START_MOVE state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf START_MOVE setzen
        send_led_command(SignalState.START_MOVE)
        
        # Sound abspielen
        play_state_sound("start_move.wav")
        
        rospy.loginfo("[START_MOVE] Start move complete, returning to IDLE")
        return 'done'

