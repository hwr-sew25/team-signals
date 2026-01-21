#!/usr/bin/env python3
"""
MOVE_FORWARD State - Roboter bewegt sich vorwärts.
"""

import smach
import rospy

from signal_project.led_engine.led_engine import send_move_direction, DIRECTION_FORWARD
from signal_project.state_machine.state_change_flag import is_state_change_requested


class MoveForwardState(smach.State):
    """
    MOVE_FORWARD State - Der Roboter bewegt sich vorwärts.
    
    LED-Segment 2 (LEDs 32-47) leuchtet hell, Rest dunkel.
    
    Outcomes:
        - 'done': State beendet
        - 'preempted': State wurde unterbrochen (neuer State kommt)
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['done', 'preempted'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        """Führt die MOVE_FORWARD-Logik aus."""
        rospy.loginfo("[MOVE_FORWARD] Entering MOVE_FORWARD state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED-Richtung setzen (Segment 2 = Vorne)
        send_move_direction(DIRECTION_FORWARD)
        
        rospy.loginfo("[MOVE_FORWARD] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt oder state_change_requested)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            # Prüfe ob Zustandswechsel angefordert wurde (verhindert Deadlocks)
            if is_state_change_requested():
                rospy.loginfo("[MOVE_FORWARD] State change requested, exiting")
                return 'preempted'
            
            rate.sleep()
        
        return 'done'

