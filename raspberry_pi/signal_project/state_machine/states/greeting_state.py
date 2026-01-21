#!/usr/bin/env python3
"""
GREETING State - Begrüßungszustand des Roboters.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound
from signal_project.state_machine.state_change_flag import is_state_change_requested


class GreetingState(smach.State):
    """
    GREETING State - Der Roboter begrüßt den Benutzer.
    
    Wartet bis ein neuer State getriggert wird.
    
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
        """Führt die GREETING-Logik aus."""
        rospy.loginfo("[GREETING] Entering GREETING state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf GREETING setzen
        send_led_command(SignalState.GREETING)
        
        # Sound abspielen
        play_state_sound("greeting.wav")
        
        rospy.loginfo("[GREETING] State active - waiting for next state trigger")
        
        # Warte bis neuer State getriggert wird
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            # Prüfe ob Zustandswechsel angefordert wurde
            if is_state_change_requested():
                rospy.loginfo("[GREETING] New state triggered, exiting")
                return 'preempted'
            
            rate.sleep()
        
        return 'done'
