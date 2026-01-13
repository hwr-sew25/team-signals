#!/usr/bin/env python3
"""
Signal Controller Node - Empfängt ROS Topics von anderen Teams und triggert States.

Dieser Node ist die zentrale Schnittstelle zwischen den anderen Teams und dem Signal-System.
Er subscribed alle relevanten Topics und triggert entsprechend die State Machine.
Zusätzlich publisht er den aktuellen State auf /signals/current_state.
"""

import rospy
from std_msgs.msg import Bool, String, Empty, UInt8, Header
from geometry_msgs.msg import Twist

from signal_project.state_machine.state_machine import create_state_machine, get_idle_state
from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import (
    calculate_direction_from_twist,
    DIRECTION_FORWARD,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    DIRECTION_BACKWARD
)

# Versuche Custom Messages zu importieren (falls gebaut)
try:
    from signals.msg import SignalState as SignalStateMsg
    from signals.msg import SignalStatusUpdate
    USE_CUSTOM_MSGS = True
    rospy.loginfo("[SIGNAL_CONTROLLER] Custom messages loaded successfully")
except ImportError:
    USE_CUSTOM_MSGS = False
    rospy.logwarn("[SIGNAL_CONTROLLER] Custom messages not available, using std_msgs")


class SignalControllerNode:
    """
    Controller Node für das Signal-System.
    
    Empfängt Nachrichten von anderen Teams und triggert die entsprechenden States.
    """
    
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
        
        # Movement Team Topics
        rospy.Subscriber('/movement/start_move', Bool, self.on_start_move)
        rospy.Subscriber('/movement/stop_move', Bool, self.on_stop_move)
        rospy.Subscriber('/movement/reverse', Bool, self.on_reverse)
        rospy.Subscriber('/movement/direction', Twist, self.on_direction)
        rospy.Subscriber('/movement/error_minor', String, self.on_error_minor)
        rospy.Subscriber('/movement/events', String, self.on_movement_event)
        rospy.Subscriber('/emergency_stop', Bool, self.on_emergency_stop)
        
        # Speech-Out Team Topic
        rospy.Subscriber('/speech_out/is_speaking', Bool, self.on_speaking)
        
        # Speech-In Team Topic
        rospy.Subscriber('/speech_in/user_intent', String, self.on_user_intent)
        
        # Directions Team Topic
        rospy.Subscriber('/directions/navigation_error', String, self.on_navigation_error)
        
        # Low Battery (von Remote Monitoring oder Hardware)
        rospy.Subscriber('/battery_state_monitoring', String, self.on_battery_state)
        
        # Display Team Topic (für Countdown/Warten)
        rospy.Subscriber('/display/countdown_started', Bool, self.on_countdown_started)
        rospy.Subscriber('/display/countdown_finished', Bool, self.on_countdown_finished)
        
        rospy.loginfo("[SIGNAL_CONTROLLER] All subscribers initialized")
        rospy.loginfo("[SIGNAL_CONTROLLER] Listening for external triggers...")
    
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
        
        Args:
            trigger_name: Name des Triggers (z.B. 'trigger_greeting')
            info: Zusätzliche Info für StatusUpdate
        """
        if self.idle_state is not None:
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Triggering: {trigger_name}")
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
        
        Empfängt Twist-Messages vom Movement-Team und steuert die LED-Richtungsanzeige:
        - LED-Segment 0 (LEDs 0-15):  Links leuchtet bei Linksdrehung
        - LED-Segment 1 (LEDs 16-31): Vorne leuchtet bei Vorwärtsfahrt
        - LED-Segment 2 (LEDs 32-47): Rechts leuchtet bei Rechtsdrehung
        - LED-Segment 3 (LEDs 48-63): Hinten leuchtet bei Rückwärtsfahrt
        """
        # Bei Bewegung: MOVE State mit Richtung aktivieren
        if msg.linear.x != 0 or msg.angular.z != 0:
            # Richtung aus Twist berechnen
            direction = calculate_direction_from_twist(msg.linear.x, msg.angular.z)
            
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
            
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Received: direction ({direction_names.get(direction, 'UNKNOWN')})")
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Twist: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
            
            # Entsprechenden MOVE State triggern
            trigger = direction_triggers.get(direction, 'trigger_move_forward')
            self.trigger_state(trigger)
        else:
            # Stillstand - zurück zu IDLE (über stop_busy)
            rospy.logdebug("[SIGNAL_CONTROLLER] Received: direction (stopped)")
    
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
        """Callback für /emergency_stop - Notaus."""
        if msg.data:
            rospy.logwarn("[SIGNAL_CONTROLLER] EMERGENCY STOP RECEIVED!")
            self.trigger_state('trigger_error_major')
    
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
    
    def on_countdown_started(self, msg):
        """Callback für /display/countdown_started - Display zeigt Countdown."""
        if msg.data:
            rospy.loginfo("[SIGNAL_CONTROLLER] Received: countdown_started - activating WAITING state")
            self.trigger_state('trigger_waiting', "Countdown auf Display gestartet")
    
    def on_countdown_finished(self, msg):
        """Callback für /display/countdown_finished - Countdown beendet."""
        if msg.data:
            rospy.loginfo("[SIGNAL_CONTROLLER] Received: countdown_finished")
            # Nach Countdown: zurück zu IDLE oder START_MOVE (je nach Kontext)
            self.trigger_state('trigger_start_move', "Countdown beendet, Fahrt startet")
    
    def run(self):
        """Startet den Controller und die State Machine."""
        import smach_ros
        import threading
        
        # Introspection Server für Debugging
        sis = smach_ros.IntrospectionServer('signal_controller_sm', self.sm, '/SIGNAL_SM')
        sis.start()
        
        # State Machine in separatem Thread ausführen
        def run_sm():
            try:
                rospy.loginfo("[SIGNAL_CONTROLLER] Starting State Machine")
                outcome = self.sm.execute()
                rospy.loginfo(f"[SIGNAL_CONTROLLER] State Machine finished: {outcome}")
            except Exception as e:
                rospy.logerr(f"[SIGNAL_CONTROLLER] State Machine error: {e}")
        
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

