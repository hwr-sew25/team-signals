#!/usr/bin/env python3
"""
IDLE State - Ruhezustand des Roboters.
"""

import smach
import rospy

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command


class IdleState(smach.State):
    """
    IDLE State - Der Roboter befindet sich im Ruhezustand.
    
    Alle States bleiben aktiv bis ein neuer State getriggert wird.
    Der neue State preempted den aktuellen State.
    
    Licht: Weiß dezent (25% absolut)
    Ton: Keiner
    
    Outcomes:
        - 'trigger_*': Wechsel zum entsprechenden State
        - 'preempted': State wurde unterbrochen (neuer State kommt)
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                'trigger_greeting',
                'trigger_idle',
                'trigger_busy',
                'trigger_stop_busy',
                'trigger_error_minor_stuck',
                'trigger_error_minor_nav',
                'trigger_room_not_found',
                'trigger_error_major',
                'trigger_low_battery',
                'trigger_move',
                'trigger_start_move',
                'trigger_stop_move',
                'trigger_goal_reached',
                'trigger_reverse',
                'trigger_speaking',
                'trigger_waiting',
                'preempted'
            ],
            input_keys=[],
            output_keys=[]
        )
        self._trigger = None

    def set_trigger(self, trigger):
        """Setzt den nächsten Trigger für den State-Wechsel."""
        self._trigger = trigger

    def execute(self, userdata):
        """Führt die IDLE-Logik aus."""
        rospy.loginfo("[IDLE] Entering IDLE state")
        
        # LED auf IDLE setzen (nur wenn kein Trigger wartet)
        if self._trigger is None:
            send_led_command(SignalState.IDLE)
        
        # Warte auf externen Trigger oder preemption
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            if self._trigger is not None:
                trigger = self._trigger
                self._trigger = None
                rospy.loginfo(f"[IDLE] Received trigger: {trigger}")
                return trigger
            
            rate.sleep()
        
        return 'preempted'
