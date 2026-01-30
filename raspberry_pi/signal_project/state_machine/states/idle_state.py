#!/usr/bin/env python3
"""
IDLE State - Ruhezustand des Roboters.

WICHTIG: Thread-Safety
Der Zugriff auf _trigger wird durch einen Lock geschützt, da:
- ROS-Callbacks (Main Thread) set_trigger() aufrufen
- State Machine (separater Thread) execute() ausführt und _trigger liest/schreibt

PRIORITÄTS-BASIERTES TRIGGERING:
- Laufender State wird NUR von P0 States (ERROR) unterbrochen
- Wartende Trigger können von höher priorisierten Triggern überschrieben werden
"""

import smach
import rospy
import threading

from signal_project.state_machine.signal_state_defs import SignalState, get_priority, Priority
from signal_project.led_engine.led_engine import send_led_command
from signal_project.state_machine.state_change_flag import clear_state_change_request

# Mapping von Trigger-Namen zu SignalStates für Prioritätsprüfung
TRIGGER_TO_STATE = {
    'trigger_greeting': SignalState.GREETING,
    'trigger_idle': SignalState.IDLE,
    'trigger_error_minor_stuck': SignalState.ERROR_MINOR_STUCK,
    'trigger_error_minor_nav': SignalState.ERROR_MINOR_NAV,
    'trigger_room_not_found': SignalState.ROOM_NOT_FOUND,
    'trigger_error_major': SignalState.ERROR_MAJOR,
    'trigger_low_battery': SignalState.LOW_BATTERY,
    'trigger_move_left': SignalState.MOVE_LEFT,
    'trigger_move_forward': SignalState.MOVE_FORWARD,
    'trigger_move_right': SignalState.MOVE_RIGHT,
    'trigger_move_backward': SignalState.MOVE_BACKWARD,
    'trigger_start_move': SignalState.START_MOVE,
    'trigger_stop_move': SignalState.STOP_MOVE,
    'trigger_goal_reached': SignalState.GOAL_REACHED,
    'trigger_speaking': SignalState.SPEAKING,
    'trigger_waiting': SignalState.WAITING,
}


def is_p0_trigger(trigger_name):
    """Prüft ob ein Trigger zu einem P0 State (ERROR) gehört."""
    state = TRIGGER_TO_STATE.get(trigger_name)
    if state is None:
        return False
    return get_priority(state) == Priority.P0


class IdleState(smach.State):
    """
    IDLE State - Der Roboter befindet sich im Ruhezustand.
    
    EINFACHE PRIORITÄTS-LOGIK:
    - Laufender State läuft IMMER weiter bis fertig
    - NUR P0 (ERROR_MAJOR, LOW_BATTERY) kann laufenden State unterbrechen
    - Erster Trigger gewinnt - nachfolgende werden ignoriert (außer P0)
    
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

    def _should_override_trigger(self, new_trigger, current_trigger):
        """
        Prüft ob der neue Trigger den wartenden Trigger überschreiben sollte.
        
        EINFACHE LOGIK:
        1. Wenn kein aktueller Trigger → überschreiben
        2. P0 (ERROR_MAJOR, LOW_BATTERY) → kann IMMER überschreiben
        3. Alle anderen → NICHT überschreiben (erster Trigger gewinnt)
        
        Args:
            new_trigger: Der neue Trigger-Name
            current_trigger: Der aktuell wartende Trigger-Name (oder None)
            
        Returns:
            bool: True wenn überschrieben werden sollte
        """
        # Kein aktueller Trigger → immer überschreiben
        if current_trigger is None:
            return True
        
        new_state = TRIGGER_TO_STATE.get(new_trigger)
        if new_state is None:
            return True  # Unbekannte Trigger erlauben
        
        # NUR P0 (ERROR_MAJOR, LOW_BATTERY) kann überschreiben
        if get_priority(new_state) == Priority.P0:
            rospy.loginfo(f"[IDLE] P0 override: {current_trigger} -> {new_trigger}")
            return True
        
        # Alle anderen: Erster Trigger gewinnt, neue werden ignoriert
        rospy.logdebug(f"[IDLE] Keeping {current_trigger}, blocking {new_trigger}")
        return False

    def set_trigger(self, trigger):
        """
        Setzt den nächsten Trigger für den State-Wechsel (thread-safe).
        
        EINFACHE LOGIK:
        - Kein Trigger wartet → Trigger setzen
        - P0 (ERROR_MAJOR, LOW_BATTERY) → kann IMMER überschreiben
        - Alle anderen → werden ignoriert wenn bereits ein Trigger wartet
        
        Wird von ROS-Callbacks im Main Thread aufgerufen.
        
        Returns:
            bool: True wenn Trigger gesetzt wurde, False wenn blockiert
        """
        with self._trigger_lock:
            if self._should_override_trigger(trigger, self._trigger):
                old_trigger = self._trigger
                self._trigger = trigger
                self._state_change_requested = True
                if old_trigger:
                    rospy.loginfo(f"[IDLE] Trigger changed: {old_trigger} -> {trigger}")
                else:
                    rospy.logdebug(f"[IDLE] Trigger set: {trigger}")
                return True
            else:
                rospy.logdebug(f"[IDLE] Trigger {trigger} blocked (current: {self._trigger})")
                return False

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
