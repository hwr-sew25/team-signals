#!/usr/bin/env python3
"""
GOAL_REACHED State - Ziel wurde erreicht.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound


class GoalReachedState(smach.State):
    """
    GOAL_REACHED State - Das Ziel wurde erreicht.
    
    Licht: Grün aufblinkend
    Ton: 2x schnelles Piepen
    
    Outcomes:
        - 'done': Ziel erreicht angezeigt, zurück zu IDLE
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
        """Führt die GOAL_REACHED-Logik aus."""
        rospy.loginfo("[GOAL_REACHED] Entering GOAL_REACHED state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf GOAL_REACHED setzen (Grün aufblinkend)
        send_led_command(SignalState.GOAL_REACHED)
        
        # Sound abspielen (2x schnelles Piepen)
        play_state_sound("goal_reached.wav")
        
        rospy.loginfo("[GOAL_REACHED] Goal reached displayed, returning to IDLE")
        return 'done'

