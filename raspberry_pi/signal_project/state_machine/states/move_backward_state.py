#!/usr/bin/env python3
"""
MOVE_BACKWARD State - Roboter bewegt sich rückwärts.
"""

import smach
import rospy

from signal_project.led_engine.led_engine import send_move_direction, DIRECTION_BACKWARD


class MoveBackwardState(smach.State):
    """
    MOVE_BACKWARD State - Der Roboter bewegt sich rückwärts.
    
    LED-Segment 0 (LEDs 0-15) leuchtet hell, Rest dunkel.
    
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
        """Führt die MOVE_BACKWARD-Logik aus."""
        rospy.loginfo("[MOVE_BACKWARD] Entering MOVE_BACKWARD state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED-Richtung setzen (Segment 0 = Hinten)
        send_move_direction(DIRECTION_BACKWARD)
        
        rospy.loginfo("[MOVE_BACKWARD] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rate.sleep()
        
        return 'done'

