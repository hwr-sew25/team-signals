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
    
    Outcomes:
        - 'trigger_greeting': Wechsel zu GREETING State
        - 'trigger_busy': Wechsel zu BUSY State
        - 'trigger_stop_busy': Wechsel zu STOP_BUSY State
        - 'trigger_error_minor_stuck': Wechsel zu ERROR_MINOR_STUCK State
        - 'trigger_error_minor_nav': Wechsel zu ERROR_MINOR_NAV State
        - 'trigger_room_not_found': Wechsel zu ROOM_NOT_FOUND State
        - 'trigger_error_major': Wechsel zu ERROR_MAJOR State
        - 'trigger_low_battery': Wechsel zu LOW_BATTERY State
        - 'trigger_start_move': Wechsel zu START_MOVE State
        - 'trigger_stop_move': Wechsel zu STOP_MOVE State
        - 'trigger_goal_reached': Wechsel zu GOAL_REACHED State
        - 'trigger_reverse': Wechsel zu REVERSE State
        - 'trigger_speaking': Wechsel zu SPEAKING State
        - 'preempted': State wurde unterbrochen
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                'trigger_greeting',
                'trigger_busy',
                'trigger_stop_busy',
                'trigger_error_minor_stuck',
                'trigger_error_minor_nav',
                'trigger_room_not_found',
                'trigger_error_major',
                'trigger_low_battery',
                'trigger_start_move',
                'trigger_stop_move',
                'trigger_goal_reached',
                'trigger_reverse',
                'trigger_speaking',
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
        
        # LED auf IDLE setzen
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
