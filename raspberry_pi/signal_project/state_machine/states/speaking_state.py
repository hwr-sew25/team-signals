#!/usr/bin/env python3
"""
SPEAKING State - Roboter spricht (Audio kommt vom Team Speech-Out).
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.state_machine.state_change_flag import is_state_change_requested


class SpeakingState(smach.State):
    """
    SPEAKING State - Roboter gibt Sprachausgabe.
    
    Der Sound wird vom Team Speech-Out abgespielt, dieser State
    zeigt nur die LED-Signalisierung (Lila konstant).
    
    Wird von Speech-Out getriggert wenn Sprachausgabe startet.
    Beenden mit STOP_SPEAKING wenn Sprachausgabe endet.
    
    Licht: Lila konstant
    Ton: Keiner (kommt von Speech-Out)
    
    Outcomes:
        - 'done': State gesetzt, zurück zu IDLE
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
        """Führt die SPEAKING-Logik aus."""
        rospy.loginfo("[SPEAKING] Entering SPEAKING state - LED only, audio from Speech-Out")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf SPEAKING setzen (Lila konstant)
        # Kein Sound - der kommt vom Team Speech-Out
        send_led_command(SignalState.SPEAKING)
        
        rospy.loginfo("[SPEAKING] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt oder state_change_requested)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            # Prüfe ob Zustandswechsel angefordert wurde (verhindert Deadlocks)
            if is_state_change_requested():
                rospy.loginfo("[SPEAKING] State change requested, exiting")
                return 'preempted'
            
            rate.sleep()
        
        return 'done'

