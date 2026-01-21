#!/usr/bin/env python3
"""
ROOM_NOT_FOUND State - Angeforderter Raum existiert nicht.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.audio_engine.audio_engine import play_state_sound
from signal_project.state_machine.state_change_flag import is_state_change_requested


class RoomNotFoundState(smach.State):
    """
    ROOM_NOT_FOUND State - Der angeforderte Raum existiert nicht.
    
    Licht: Rot (kurz)
    Ton: 1x piep
    
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
        """Führt die ROOM_NOT_FOUND-Logik aus."""
        rospy.loginfo("[ROOM_NOT_FOUND] Entering ROOM_NOT_FOUND state")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # LED auf ROOM_NOT_FOUND setzen (Rot kurz)
        send_led_command(SignalState.ROOM_NOT_FOUND)
        
        # Sound abspielen (1x piep)
        play_state_sound("room_not_found.wav")
        
        rospy.loginfo("[ROOM_NOT_FOUND] State active - waiting for next state")
        
        # Warte bis neuer State kommt (preempt oder state_change_requested)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            # Prüfe ob Zustandswechsel angefordert wurde (verhindert Deadlocks)
            if is_state_change_requested():
                rospy.loginfo("[ROOM_NOT_FOUND] State change requested, exiting")
                return 'preempted'
            
            rate.sleep()
        
        return 'done'

