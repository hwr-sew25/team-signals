#!/usr/bin/env python3
"""
IDLE State - Ruhezustand des Roboters.

WICHTIG: Thread-Safety
Der Zugriff auf _trigger wird durch einen Lock geschützt, da:
- ROS-Callbacks (Main Thread) set_trigger() aufrufen
- State Machine (separater Thread) execute() ausführt und _trigger liest/schreibt
"""

import smach
import rospy
import threading

from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command
from signal_project.state_machine.state_change_flag import clear_state_change_request


class IdleState(smach.State):
    """
    IDLE State - Der Roboter befindet sich im Ruhezustand.
    
    Alle States bleiben aktiv bis ein neuer State getriggert wird.
    Der neue State preempted den aktuellen State.
    
    Licht: Weiß dezent (25% absolut)
    Ton: Keiner
    
    WICHTIG: Thread-Safety
    Der Zugriff auf _trigger ist durch _trigger_lock geschützt,
    um Race Conditions zwischen ROS-Callbacks und State Machine Thread zu vermeiden.
    
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
                'trigger_move_left',
                'trigger_move_forward',
                'trigger_move_right',
                'trigger_move_backward',
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
        # Thread-Lock für sicheren Zugriff auf _trigger
        self._trigger_lock = threading.Lock()
        # Flag das andere States prüfen können um schnell zu reagieren
        self._state_change_requested = False

    def set_trigger(self, trigger):
        """
        Setzt den nächsten Trigger für den State-Wechsel (thread-safe).
        Setzt auch das state_change_requested Flag, damit andere States
        schnell reagieren können.
        
        Wird von ROS-Callbacks im Main Thread aufgerufen.
        """
        with self._trigger_lock:
            self._trigger = trigger
            self._state_change_requested = True
        rospy.logdebug(f"[IDLE] Trigger set: {trigger}, state_change_requested=True")

    def is_state_change_requested(self):
        """
        Prüft ob ein Zustandswechsel angefordert wurde (thread-safe).
        Kann von anderen States aufgerufen werden.
        """
        with self._trigger_lock:
            return self._state_change_requested

    def has_pending_trigger(self):
        """Prüft ob ein Trigger wartet (thread-safe)."""
        with self._trigger_lock:
            return self._trigger is not None

    def _get_and_clear_trigger(self):
        """
        Holt den aktuellen Trigger und setzt ihn zurück (thread-safe).
        Atomare Operation um Race Conditions zu vermeiden.
        
        Returns:
            Der Trigger-String oder None wenn kein Trigger gesetzt war
        """
        with self._trigger_lock:
            trigger = self._trigger
            if trigger is not None:
                self._trigger = None
                self._state_change_requested = False
            return trigger

    def execute(self, userdata):
        """Führt die IDLE-Logik aus."""
        rospy.loginfo("[IDLE] Entering IDLE state")
        
        # Globales state_change_requested Flag zurücksetzen
        clear_state_change_request()
        with self._trigger_lock:
            self._state_change_requested = False
        
        # LED auf IDLE setzen (nur wenn kein Trigger wartet)
        with self._trigger_lock:
            has_trigger = self._trigger is not None
        if not has_trigger:
            send_led_command(SignalState.IDLE)
        
        # Warte auf externen Trigger oder preemption
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            # Thread-sicheres Holen und Zurücksetzen des Triggers
            trigger = self._get_and_clear_trigger()
            if trigger is not None:
                rospy.loginfo(f"[IDLE] Received trigger: {trigger}")
                return trigger
            
            rate.sleep()
        
        return 'preempted'
