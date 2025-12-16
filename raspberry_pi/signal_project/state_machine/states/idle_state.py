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
        - 'trigger_error': Wechsel zu ERROR State
        - 'trigger_low_battery': Wechsel zu LOW_BATTERY State
        - 'trigger_busy': Wechsel zu BUSY State
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                'trigger_greeting',
                'trigger_error',
                'trigger_low_battery',
                'trigger_busy',
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

