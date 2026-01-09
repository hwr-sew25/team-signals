#!/usr/bin/env python3
"""
Signal Controller Node - Empfängt ROS Topics von anderen Teams und triggert States.

Dieser Node ist die zentrale Schnittstelle zwischen den anderen Teams und dem Signal-System.
Er subscribed alle relevanten Topics und triggert entsprechend die State Machine.
"""

import rospy
from std_msgs.msg import Bool, String, Empty
from geometry_msgs.msg import Twist

from signal_project.state_machine.state_machine import create_state_machine, get_idle_state


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
        
        rospy.loginfo("[SIGNAL_CONTROLLER] All subscribers initialized")
        rospy.loginfo("[SIGNAL_CONTROLLER] Listening for external triggers...")
    
    # === CALLBACK FUNKTIONEN ===
    
    def trigger_state(self, trigger_name):
        """Hilfsfunktion zum Triggern eines States."""
        if self.idle_state is not None:
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Triggering: {trigger_name}")
            self.idle_state.set_trigger(trigger_name)
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
        """Callback für /movement/direction - Bewegungsrichtung."""
        # Bei Bewegung: Busy State aktivieren
        if msg.linear.x != 0 or msg.angular.z != 0:
            rospy.loginfo("[SIGNAL_CONTROLLER] Received: direction (moving)")
            self.trigger_state('trigger_busy')
    
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

