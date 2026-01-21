#!/usr/bin/env python3
"""
START_MOVE State - Bewegung beginnt.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound
from signal_project.state_machine.state_change_flag import is_state_change_requested


class StartMoveState(smach.State):
    """
    START_MOVE State - Roboter beginnt Bewegung.
    
    Wartet bis ein neuer State getriggert wird (z.B. MOVE_FORWARD).
    
    Outcomes:
        - 'done': Zurück zu IDLE (wenn kein neuer Trigger kommt)
        - 'preempted': Neuer State wurde getriggert
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
        
        rospy.loginfo("[START_MOVE] State active - waiting for next state trigger")
        
        # Warte bis neuer State getriggert wird
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            # Prüfe ob Zustandswechsel angefordert wurde
            if is_state_change_requested():
                rospy.loginfo("[START_MOVE] New state triggered, exiting")
                return 'preempted'
            
            rate.sleep()
        
        return 'done'
