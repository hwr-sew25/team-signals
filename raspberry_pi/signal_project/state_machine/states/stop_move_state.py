#!/usr/bin/env python3
"""
STOP_MOVE State - Bewegung endet.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound


class StopMoveState(smach.State):
    """
    STOP_MOVE State - Roboter stoppt Bewegung.
    
    Outcomes:
        - 'done': Bewegungsstopp signalisiert, zurück zu IDLE
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
        """Führt die STOP_MOVE-Logik aus."""
        rospy.loginfo("[STOP_MOVE] Entering STOP_MOVE state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf STOP_MOVE setzen
        send_led_command(SignalState.STOP_MOVE)
        
        # Sound abspielen
        play_state_sound("stop_move.wav")
        
        rospy.loginfo("[STOP_MOVE] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rate.sleep()
        
        return 'done'

