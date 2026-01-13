#!/usr/bin/env python3
"""
STOP_BUSY State - Busy-Zustand beenden.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command


class StopBusyState(smach.State):
    """
    STOP_BUSY State - Busy-Zustand wird beendet.
    
    Licht: Zurück zu Idle-Pattern (weiß, weich konstant)
    Ton: Kein Ton
    
    Outcomes:
        - 'done': Transition abgeschlossen, zurück zu IDLE
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
        """Führt die STOP_BUSY-Logik aus."""
        rospy.loginfo("[STOP_BUSY] Entering STOP_BUSY state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf STOP_BUSY setzen (weiß, weich konstant - wie Idle)
        send_led_command(SignalState.STOP_BUSY)
        
        # Kein Sound bei Stop-Busy
        
        rospy.loginfo("[STOP_BUSY] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rate.sleep()
        
        return 'done'

