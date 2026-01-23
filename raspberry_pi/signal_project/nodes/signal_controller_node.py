#!/usr/bin/env python3
"""
Signal Controller Node - Empfängt ROS Topics von anderen Teams und triggert States.

Dieser Node ist die zentrale Schnittstelle zwischen den anderen Teams und dem Signal-System.
Er subscribed alle relevanten Topics und triggert entsprechend die State Machine.
Zusätzlich publisht er den aktuellen State auf /signals/current_state.

FIXES IMPLEMENTIERT:
- Emergency Stop Release-Handling
- State Machine Thread-Crash Recovery
- /movement/direction Debouncing (verhindert Nachrichtenflut)
- Subscriber Queue Sizes
- Graceful Shutdown
"""

import rospy
from std_msgs.msg import Bool, String, Empty, UInt8, Header
from geometry_msgs.msg import Twist

from signal_project.state_machine.state_machine import create_state_machine, get_idle_state
from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.state_machine.state_change_flag import request_state_change
from signal_project.led_engine.led_engine import (
    calculate_direction_from_twist,
    DIRECTION_FORWARD,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    DIRECTION_BACKWARD
)

# Versuche Custom Messages zu importieren (falls gebaut)
# HINWEIS: print() statt rospy.log() da rospy.init_node() noch nicht aufgerufen wurde
try:
    from signals.msg import SignalState as SignalStateMsg
    from signals.msg import SignalStatusUpdate
    USE_CUSTOM_MSGS = True
    print("[SIGNAL_CONTROLLER] Custom messages loaded successfully")
except ImportError:
    USE_CUSTOM_MSGS = False
    print("[SIGNAL_CONTROLLER] Custom messages not available, using std_msgs")


class SignalControllerNode:
    """
    Controller Node für das Signal-System.
    
    Empfängt Nachrichten von anderen Teams und triggert die entsprechenden States.
    """
    
    # Debounce-Zeit für direction Messages (in Sekunden)
    DIRECTION_DEBOUNCE_SEC = 0.2
    
    def __init__(self):
        """Initialisiert den Controller Node."""
        rospy.init_node('signal_controller', anonymous=False)
        rospy.loginfo("[SIGNAL_CONTROLLER] Initializing Signal Controller Node")
        
        # State Machine erstellen
        self.sm = create_state_machine()
        self.idle_state = get_idle_state(self.sm)
        
        if self.idle_state is None:
            rospy.logerr("[SIGNAL_CONTROLLER] IDLE State not found!")
            return
        
        # Aktueller State für Publishing
        self.current_state = SignalState.IDLE
        self.previous_state = SignalState.IDLE
        self.state_changed_time = rospy.Time.now()
        
        # Debouncing für /movement/direction
        self._last_direction = None
        self._last_direction_time = rospy.Time.now()
        
        # Emergency Stop Status tracking
        self._emergency_stop_active = False
        
        # === PUBLISHERS für andere Teams ===
        
        # Publisher für aktuellen State (als String für einfache Integration)
        self.state_pub = rospy.Publisher('/signals/current_state', String, queue_size=10)
        self.state_id_pub = rospy.Publisher('/signals/current_state_id', UInt8, queue_size=10)
        
        # Falls Custom Messages verfügbar sind
        if USE_CUSTOM_MSGS:
            self.custom_state_pub = rospy.Publisher(
                '/signals/state', SignalStateMsg, queue_size=10
            )
            self.status_update_pub = rospy.Publisher(
                '/signals/status_update', SignalStatusUpdate, queue_size=10
            )
        
        rospy.loginfo("[SIGNAL_CONTROLLER] Publishers initialized:")
        rospy.loginfo("[SIGNAL_CONTROLLER]   - /signals/current_state (String)")
        rospy.loginfo("[SIGNAL_CONTROLLER]   - /signals/current_state_id (UInt8)")
        
        # === SUBSCRIBERS für Topics von anderen Teams ===
        # HINWEIS: queue_size explizit gesetzt für wichtige Topics
        
        # Movement Team Topics
        rospy.Subscriber('/movement/start_move', Bool, self.on_start_move, queue_size=5)
        rospy.Subscriber('/movement/stop_move', Bool, self.on_stop_move, queue_size=5)
        rospy.Subscriber('/movement/reverse', Bool, self.on_reverse, queue_size=5)
        rospy.Subscriber('/movement/direction', Twist, self.on_direction, queue_size=1)  # Nur neueste
        rospy.Subscriber('/movement/error_minor', String, self.on_error_minor, queue_size=10)
        rospy.Subscriber('/movement/events', String, self.on_movement_event, queue_size=10)
        rospy.Subscriber('/emergency_stop', Bool, self.on_emergency_stop, queue_size=10)  # Wichtig!
        
        # Speech-Out Team Topic
        rospy.Subscriber('/speech_out/is_speaking', Bool, self.on_speaking, queue_size=5)
        
        # Speech-In Team Topic
        rospy.Subscriber('/speech_in/user_intent', String, self.on_user_intent, queue_size=10)
        
        # Directions Team Topic
        rospy.Subscriber('/directions/navigation_error', String, self.on_navigation_error, queue_size=10)
        
        # Low Battery (von Remote Monitoring oder Hardware)
        rospy.Subscriber('/battery_state_monitoring', String, self.on_battery_state, queue_size=5)
        
        # Display Team Topic (für Start/Stop Druecken)
        rospy.Subscriber('/display/start_druecken', Bool, self.on_start_druecken, queue_size=5)
        rospy.Subscriber('/display/stop_druecken', Bool, self.on_stop_druecken, queue_size=5)
        
        # Shutdown-Handler registrieren
        rospy.on_shutdown(self.cleanup)
        
        rospy.loginfo("[SIGNAL_CONTROLLER] All subscribers initialized")
        rospy.loginfo("[SIGNAL_CONTROLLER] Listening for external triggers...")
    
    def cleanup(self):
        """
        Graceful Shutdown - Aufräumen bei Node-Beendigung.
        Schließt Serial-Verbindung und stoppt laufende Sounds.
        """
        rospy.loginfo("[SIGNAL_CONTROLLER] Shutdown requested - cleaning up...")
        
        try:
            # Audio stoppen
            from signal_project.audio_engine.audio_engine import stop_current_sound
            stop_current_sound()
            rospy.loginfo("[SIGNAL_CONTROLLER] Audio stopped")
        except Exception as e:
            rospy.logwarn(f"[SIGNAL_CONTROLLER] Error stopping audio: {e}")
        
        try:
            # Serial-Verbindung schließen
            from signal_project.led_engine.led_engine import cleanup_connection
            cleanup_connection()
            rospy.loginfo("[SIGNAL_CONTROLLER] Serial connection closed")
        except Exception as e:
            rospy.logwarn(f"[SIGNAL_CONTROLLER] Error closing serial: {e}")
        
        rospy.loginfo("[SIGNAL_CONTROLLER] Cleanup complete")
    
    # === CALLBACK FUNKTIONEN ===
    
    def publish_state(self, state: SignalState, info: str = ""):
        """
        Publisht den aktuellen State auf allen relevanten Topics.
        
        Args:
            state: Der neue SignalState
            info: Zusätzliche Info (z.B. Fehlerdetails)
        """
        # State speichern
        self.previous_state = self.current_state
        self.current_state = state
        self.state_changed_time = rospy.Time.now()
        
        # Standard Messages publishen
        self.state_pub.publish(String(data=state.name))
        self.state_id_pub.publish(UInt8(data=state.value))
        
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Published state: {state.name} (ID: {state.value})")
        
        # Custom Messages publishen (falls verfügbar)
        if USE_CUSTOM_MSGS:
            try:
                # SignalState Message
                state_msg = SignalStateMsg()
                state_msg.header = Header()
                state_msg.header.stamp = rospy.Time.now()
                state_msg.state_id = state.value
                state_msg.state_name = state.name
                state_msg.led_active = True
                state_msg.audio_active = False  # Wird später gesetzt
                self.custom_state_pub.publish(state_msg)
                
                # StatusUpdate Message
                update_msg = SignalStatusUpdate()
                update_msg.header = Header()
                update_msg.header.stamp = rospy.Time.now()
                update_msg.current_state = state.value
                update_msg.current_state_name = state.name
                update_msg.previous_state = self.previous_state.value
                update_msg.previous_state_name = self.previous_state.name
                update_msg.state_changed_at = self.state_changed_time
                update_msg.info = info
                self.status_update_pub.publish(update_msg)
            except Exception as e:
                rospy.logwarn(f"[SIGNAL_CONTROLLER] Error publishing custom msg: {e}")
    
    # Mapping von Trigger zu SignalState
    TRIGGER_TO_STATE = {
        'trigger_greeting': SignalState.GREETING,
        'trigger_idle': SignalState.IDLE,
        'trigger_busy': SignalState.BUSY,
        'trigger_stop_busy': SignalState.STOP_BUSY,
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
        'trigger_reverse': SignalState.REVERSE,
        'trigger_speaking': SignalState.SPEAKING,
        'trigger_waiting': SignalState.WAITING,
    }
    
    def trigger_state(self, trigger_name, info: str = ""):
        """
        Hilfsfunktion zum Triggern eines States.
        Publisht automatisch den neuen State.
        
        WICHTIG: Diese Methode setzt den Trigger im IdleState UND signalisiert
        über das globale state_change_flag, dass ein Zustandswechsel angefordert
        wurde. Alle States prüfen dieses Flag in ihren while-Schleifen und 
        beenden sich, wenn es gesetzt ist. Damit werden Deadlocks vermieden.
        
        Args:
            trigger_name: Name des Triggers (z.B. 'trigger_greeting')
            info: Zusätzliche Info für StatusUpdate
        """
        if self.idle_state is not None:
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Triggering: {trigger_name}")
            
            # WICHTIG: Zuerst das globale Flag setzen, damit alle States reagieren
            request_state_change()
            
            # Dann den Trigger im IdleState setzen
            self.idle_state.set_trigger(trigger_name)
            
            # State publishen
            if trigger_name in self.TRIGGER_TO_STATE:
                self.publish_state(self.TRIGGER_TO_STATE[trigger_name], info)
        else:
            rospy.logwarn("[SIGNAL_CONTROLLER] Cannot trigger - IDLE state not available")
    
    def on_start_move(self, msg):
        """Callback für /movement/start_move - Bewegung startet."""
        if msg.data:
            rospy.loginfo("[SIGNAL_CONTROLLER] Received: start_move")
            self.trigger_state('trigger_start_move')
    
    def on_stop_move(self, msg):
        """Callback für /movement/stop_move - Bewegung stoppt."""
        if msg.data:
            rospy.loginfo("[SIGNAL_CONTROLLER] Received: stop_move")
            self.trigger_state('trigger_stop_move')
    
    def on_reverse(self, msg):
        """Callback für /movement/reverse - Rückwärtsfahren."""
        if msg.data:
            rospy.loginfo("[SIGNAL_CONTROLLER] Received: reverse")
            self.trigger_state('trigger_reverse')
    
    def on_direction(self, msg):
        """
        Callback für /movement/direction - Bewegungsrichtung.
        
        WICHTIG: Implementiert Debouncing um Nachrichtenflut zu vermeiden.
        Triggert nur wenn sich die Richtung ändert oder die Debounce-Zeit abgelaufen ist.
        
        Empfängt Twist-Messages vom Movement-Team und steuert die LED-Richtungsanzeige:
        - LED-Segment 0 (LEDs 0-15):  Rückwärts
        - LED-Segment 1 (LEDs 16-31): Rechts
        - LED-Segment 2 (LEDs 32-47): Vorwärts
        - LED-Segment 3 (LEDs 48-63): Links
        """
        # Bei Bewegung: MOVE State mit Richtung aktivieren
        if msg.linear.x != 0 or msg.angular.z != 0:
            # Richtung aus Twist berechnen
            direction = calculate_direction_from_twist(msg.linear.x, msg.angular.z)
            
            # DEBOUNCING: Nur triggern wenn Richtung anders oder Zeit abgelaufen
            now = rospy.Time.now()
            time_since_last = (now - self._last_direction_time).to_sec()
            
            if direction != self._last_direction or time_since_last > self.DIRECTION_DEBOUNCE_SEC:
                self._last_direction = direction
                self._last_direction_time = now
                
                # Mapping von Richtung zu Trigger
                direction_triggers = {
                    DIRECTION_LEFT: 'trigger_move_left',
                    DIRECTION_FORWARD: 'trigger_move_forward',
                    DIRECTION_RIGHT: 'trigger_move_right',
                    DIRECTION_BACKWARD: 'trigger_move_backward'
                }
                
                direction_names = {
                    DIRECTION_LEFT: "LEFT",
                    DIRECTION_FORWARD: "FORWARD",
                    DIRECTION_RIGHT: "RIGHT",
                    DIRECTION_BACKWARD: "BACKWARD"
                }
                
                rospy.loginfo(f"[SIGNAL_CONTROLLER] Direction: {direction_names.get(direction, 'UNKNOWN')}")
                
                # Entsprechenden MOVE State triggern
                trigger = direction_triggers.get(direction, 'trigger_move_forward')
                self.trigger_state(trigger)
        else:
            # Stillstand - Richtung zurücksetzen
            if self._last_direction is not None:
                self._last_direction = None
                rospy.logdebug("[SIGNAL_CONTROLLER] Direction stopped")
    
    def on_error_minor(self, msg):
        """Callback für /movement/error_minor - Leichter Fehler."""
        error_type = msg.data.lower()
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Received: error_minor ({error_type})")
        
        if 'stuck' in error_type:
            self.trigger_state('trigger_error_minor_stuck')
        elif 'nav' in error_type or 'navigation' in error_type:
            self.trigger_state('trigger_error_minor_nav')
        else:
            # Default: Stuck
            self.trigger_state('trigger_error_minor_stuck')
    
    def on_movement_event(self, msg):
        """Callback für /movement/events - Bewegungs-Events."""
        event = msg.data.lower()
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Received: movement_event ({event})")
        
        if 'goal_reached' in event or 'arrived' in event:
            self.trigger_state('trigger_goal_reached')
        elif 'start' in event:
            self.trigger_state('trigger_start_move')
        elif 'stop' in event:
            self.trigger_state('trigger_stop_move')
    
    def on_emergency_stop(self, msg):
        """
        Callback für /emergency_stop - Notaus.
        
        WICHTIG: Behandelt sowohl Aktivierung als auch Deaktivierung des Notaus!
        - msg.data == True:  Notaus aktiviert → ERROR_MAJOR
        - msg.data == False: Notaus gelöst → zurück zu IDLE
        """
        if msg.data:
            # Notaus AKTIVIERT
            if not self._emergency_stop_active:
                rospy.logwarn("[SIGNAL_CONTROLLER] !! EMERGENCY STOP ACTIVATED !!")
                self._emergency_stop_active = True
                self.trigger_state('trigger_error_major', "Emergency Stop activated")
        else:
            # Notaus GELÖST
            if self._emergency_stop_active:
                rospy.loginfo("[SIGNAL_CONTROLLER] Emergency stop RELEASED - returning to IDLE")
                self._emergency_stop_active = False
                self.trigger_state('trigger_idle', "Emergency Stop released")
    
    def on_speaking(self, msg):
        """Callback für /speech_out/is_speaking - Roboter spricht."""
        if msg.data:
            rospy.loginfo("[SIGNAL_CONTROLLER] Received: is_speaking (True)")
            self.trigger_state('trigger_speaking')
        else:
            rospy.loginfo("[SIGNAL_CONTROLLER] Received: is_speaking (False)")
            # Wenn Sprechen beendet, zurück zu Stop-Busy (sanfter Übergang)
            self.trigger_state('trigger_stop_busy')
    
    def on_user_intent(self, msg):
        """Callback für /speech_in/user_intent - Benutzer-Absicht erkannt."""
        intent = msg.data.lower()
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Received: user_intent ({intent})")
        
        if 'greeting' in intent or 'hello' in intent or 'hallo' in intent:
            self.trigger_state('trigger_greeting')
        # Weitere Intents können hier hinzugefügt werden
    
    def on_navigation_error(self, msg):
        """Callback für /directions/navigation_error - Navigationsfehler."""
        error = msg.data.lower()
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Received: navigation_error ({error})")
        
        if 'room_not_found' in error or 'nicht gefunden' in error or 'not found' in error:
            self.trigger_state('trigger_room_not_found')
        else:
            self.trigger_state('trigger_error_minor_nav')
    
    def on_battery_state(self, msg):
        """Callback für /battery_state_monitoring - Batteriestatus."""
        state = msg.data.lower()
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Received: battery_state ({state})")
        
        if 'low' in state or 'critical' in state:
            self.trigger_state('trigger_low_battery')
    
    def on_start_druecken(self, msg):
        """Callback für /display/start_druecken - Benutzer soll Start drücken."""
        if msg.data:
            rospy.loginfo("[SIGNAL_CONTROLLER] Received: start_druecken - activating WAITING state")
            self.trigger_state('trigger_waiting', "Warte auf Start-Knopf")
    
    def on_stop_druecken(self, msg):
        """Callback für /display/stop_druecken - Benutzer soll Stop drücken."""
        if msg.data:
            rospy.loginfo("[SIGNAL_CONTROLLER] Received: stop_druecken")
            # Nach Stop-Druecken: zurück zu IDLE oder START_MOVE (je nach Kontext)
            self.trigger_state('trigger_start_move', "Stop gedrückt, Fahrt startet")
    
    def run(self):
        """Startet den Controller und die State Machine."""
        import smach_ros
        import threading
        
        # Introspection Server für Debugging
        sis = smach_ros.IntrospectionServer('signal_controller_sm', self.sm, '/SIGNAL_SM')
        sis.start()
        
        # State Machine in separatem Thread ausführen MIT RECOVERY
        def run_sm():
            """
            State Machine Hauptloop mit automatischer Recovery bei Crashes.
            """
            while not rospy.is_shutdown():
                try:
                    rospy.loginfo("[SIGNAL_CONTROLLER] Starting State Machine")
                    outcome = self.sm.execute()
                    rospy.loginfo(f"[SIGNAL_CONTROLLER] State Machine finished: {outcome}")
                    
                    if outcome == 'shutdown':
                        break
                        
                except Exception as e:
                    rospy.logerr(f"[SIGNAL_CONTROLLER] State Machine CRASHED: {e}")
                    rospy.logwarn("[SIGNAL_CONTROLLER] Restarting State Machine in 2 seconds...")
                    
                    # Kurz warten bevor Neustart
                    rospy.sleep(2.0)
                    
                    # State Machine neu erstellen
                    try:
                        self.sm = create_state_machine()
                        self.idle_state = get_idle_state(self.sm)
                        rospy.loginfo("[SIGNAL_CONTROLLER] State Machine recreated successfully")
                    except Exception as e2:
                        rospy.logerr(f"[SIGNAL_CONTROLLER] Failed to recreate State Machine: {e2}")
                        rospy.sleep(5.0)  # Länger warten bei schwerem Fehler
        
        sm_thread = threading.Thread(target=run_sm, daemon=True)
        sm_thread.start()
        
        # ROS Spin
        rospy.loginfo("[SIGNAL_CONTROLLER] Controller running. Waiting for messages...")
        rospy.spin()
        
        # Cleanup
        sis.stop()
        rospy.loginfo("[SIGNAL_CONTROLLER] Shutdown complete")


def main():
    """Hauptfunktion."""
    try:
        controller = SignalControllerNode()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[SIGNAL_CONTROLLER] ROS Interrupt received")


if __name__ == '__main__':
    main()
