#!/usr/bin/env python3
"""
MOVE_LEFT State - Roboter bewegt sich nach links.
"""

import smach
import rospy

from signal_project.led_engine.led_engine import send_move_direction, DIRECTION_LEFT


class MoveLeftState(smach.State):
    """
    MOVE_LEFT State - Der Roboter bewegt sich nach links.
    
    LED-Segment 3 (LEDs 48-63) leuchtet hell, Rest dunkel.
    
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
        """Führt die MOVE_LEFT-Logik aus."""
        rospy.loginfo("[MOVE_LEFT] Entering MOVE_LEFT state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED-Richtung setzen (Segment 3 = Links)
        send_move_direction(DIRECTION_LEFT)
        
        rospy.loginfo("[MOVE_LEFT] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rate.sleep()
        
        return 'done'

