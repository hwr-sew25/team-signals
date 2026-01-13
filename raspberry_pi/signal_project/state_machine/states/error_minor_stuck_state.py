#!/usr/bin/env python3
"""
ERROR_MINOR_STUCK State - Roboter ist steckengeblieben.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound


class ErrorMinorStuckState(smach.State):
    """
    ERROR_MINOR_STUCK State - Roboter ist steckengeblieben (Stuck).
    
    Licht: Gelb durchgehend
    Ton: 3x kurz piepen, regelmäßig
    
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
        """Führt die ERROR_MINOR_STUCK-Logik aus."""
        rospy.loginfo("[ERROR_MINOR_STUCK] Entering ERROR_MINOR_STUCK state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf ERROR_MINOR_STUCK setzen (Gelb durchgehend)
        send_led_command(SignalState.ERROR_MINOR_STUCK)
        
        # Sound abspielen (3x kurz piepen)
        play_state_sound("error_minor_stuck.wav")
        
        rospy.loginfo("[ERROR_MINOR_STUCK] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rate.sleep()
        
        return 'done'

