#!/usr/bin/env python3
"""
BUSY State - Roboter ist beschäftigt.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.state_machine.state_change_flag import is_state_change_requested


class BusyState(smach.State):
    """
    BUSY State - Der Roboter ist beschäftigt (z.B. verarbeitet Anfrage).
    
    Licht: Blau konstant
    Ton: Kein Ton
    
    Outcomes:
        - 'done': Busy beendet, zurück zu IDLE
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
        """Führt die BUSY-Logik aus."""
        rospy.loginfo("[BUSY] Entering BUSY state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf BUSY setzen (Blau konstant)
        send_led_command(SignalState.BUSY)
        
        # Kein Sound bei Busy
        
        rospy.loginfo("[BUSY] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt oder state_change_requested)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            # Prüfe ob Zustandswechsel angefordert wurde (verhindert Deadlocks)
            if is_state_change_requested():
                rospy.loginfo("[BUSY] State change requested, exiting")
                return 'preempted'
            
            rate.sleep()
        
        return 'done'

