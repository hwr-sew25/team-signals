#!/usr/bin/env python3
"""
MOVE_RIGHT State - Roboter bewegt sich nach rechts.
"""

import smach
import rospy

from signal_project.led_engine.led_engine import send_move_direction, DIRECTION_RIGHT


class MoveRightState(smach.State):
    """
    MOVE_RIGHT State - Der Roboter bewegt sich nach rechts.
    
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
        """Führt die MOVE_RIGHT-Logik aus."""
        rospy.loginfo("[MOVE_RIGHT] Entering MOVE_RIGHT state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED-Richtung setzen (Segment 2 = Rechts)
        send_move_direction(DIRECTION_RIGHT)
        
        rospy.loginfo("[MOVE_RIGHT] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rate.sleep()
        
        return 'done'

