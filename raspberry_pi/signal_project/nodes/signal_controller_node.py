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
import math
from std_msgs.msg import Bool, String, Empty, UInt8, Header
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

from signal_project.state_machine.state_machine import create_state_machine, get_idle_state
from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.state_machine.state_change_flag import request_state_change
from signal_project.led_engine.led_engine import (
    DIRECTION_FORWARD,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    DIRECTION_BACKWARD
)
from signal_project.audio_engine.audio_engine import set_volume_for_speaking

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

# Versuche Speech Team Messages zu importieren
try:
    from speech_in.msg import SpeechStatus
    USE_SPEECH_MSGS = True
    print("[SIGNAL_CONTROLLER] Speech messages (speech_in) loaded successfully")
except ImportError:
    USE_SPEECH_MSGS = False
    print("[SIGNAL_CONTROLLER] Speech messages not available, using String fallback")

# Versuche Movement Team Messages zu importieren
try:
    from movement_api.msg import NavStatus, EmergencyStop, TargetPose
    USE_MOVEMENT_MSGS = True
    print("[SIGNAL_CONTROLLER] Movement messages (movement_api) loaded successfully")
except ImportError:
    USE_MOVEMENT_MSGS = False
    print("[SIGNAL_CONTROLLER] Movement messages not available, using String fallback")

# NavStatus Konstanten (falls movement_api nicht verfügbar)
class NavStatusConstants:
    READY = 0
    MOVING_TO_TARGET = 1
    ARRIVED = 2
    WAITING_FOR_SPEECH = 3
    RETURNING_TO_START = 4
    AT_START = 5
    EMERGENCY_STOP = 6
    FAILED = 7


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
        
        # Debouncing für Richtungsänderungen
        self._last_direction = None
        self._last_direction_time = rospy.Time.now()
        
        # Position-Tracking für Bewegungsrichtungserkennung
        self._last_position_x = None
        self._last_position_y = None
        self._last_position_time = None
        self._is_moving = False
        
        # Schwellenwerte für Bewegungserkennung
        self.POSITION_CHANGE_THRESHOLD = 0.05  # Mindestbewegung in Metern
        self.DIRECTION_ANGLE_TOLERANCE = 45.0  # Grad für Richtungszuordnung
        
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
        
        # Movement Team Topics (Team Movement API)
        # /navbot/target_pose - Zielposition (movement_api/TargetPose)
        if USE_MOVEMENT_MSGS:
            rospy.Subscriber('/navbot/target_pose', TargetPose, self.on_target_pose, queue_size=5)
            rospy.loginfo("[SIGNAL_CONTROLLER] Subscribed to /navbot/target_pose (TargetPose)")
        else:
            rospy.Subscriber('/navbot/target_pose', PoseStamped, self.on_target_pose_fallback, queue_size=5)
            rospy.loginfo("[SIGNAL_CONTROLLER] Subscribed to /navbot/target_pose (PoseStamped fallback)")
        
        # /navbot/nav_status - Navigationsstatus (movement_api/NavStatus)
        # Hier kommt GOAL_REACHED ("Arrived") her!
        if USE_MOVEMENT_MSGS:
            rospy.Subscriber('/navbot/nav_status', NavStatus, self.on_nav_status, queue_size=10)
            rospy.loginfo("[SIGNAL_CONTROLLER] Subscribed to /navbot/nav_status (NavStatus)")
        else:
            rospy.Subscriber('/navbot/nav_status', String, self.on_nav_status_string, queue_size=10)
            rospy.loginfo("[SIGNAL_CONTROLLER] Subscribed to /navbot/nav_status (String fallback)")
        
        # /cmd_vel - Für Bewegungsrichtungserkennung
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd_vel, queue_size=1)  # Aktuelle Geschwindigkeit
        # Wir subscriben zusätzlich auf die Odometrie für Positionstracking
        rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=1)
        
        # Emergency Stop (movement_api/EmergencyStop)
        if USE_MOVEMENT_MSGS:
            rospy.Subscriber('/emergency_stop', EmergencyStop, self.on_emergency_stop, queue_size=10)
            rospy.loginfo("[SIGNAL_CONTROLLER] Subscribed to /emergency_stop (EmergencyStop)")
        else:
            rospy.Subscriber('/emergency_stop', Bool, self.on_emergency_stop_bool, queue_size=10)
            rospy.loginfo("[SIGNAL_CONTROLLER] Subscribed to /emergency_stop (Bool fallback)")
        
        # Speech-Out Team Topic (Roboter spricht)
        # Für Lautstärke-Steuerung: Wenn Roboter spricht → unsere Sounds leiser
        rospy.Subscriber('/speech_out/is_speaking', Bool, self.on_speech_out_speaking, queue_size=5)
        
        # Speech-In Team Topic (Spracherkennung / User Input)
        # Für Room Not Found, Greetings, etc. erkennen
        if USE_SPEECH_MSGS:
            rospy.Subscriber('/speech_output', SpeechStatus, self.on_speech_in_status, queue_size=10)
            rospy.loginfo("[SIGNAL_CONTROLLER] Subscribed to /speech_output (SpeechStatus from speech_in)")
        else:
            # Fallback: String-basierter Subscriber
            rospy.Subscriber('/speech_output', String, self.on_speech_in_status_string, queue_size=10)
            rospy.loginfo("[SIGNAL_CONTROLLER] Subscribed to /speech_output (String fallback)")
        
        # Directions Team Topic
        rospy.Subscriber('/directions/navigation_error', String, self.on_navigation_error, queue_size=10)
        
        # Low Battery (von Remote Monitoring)
        rospy.Subscriber('monitoring/battery_state', String, self.on_battery_state, queue_size=5)
        
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
    
    def calculate_direction_from_position_change(self, dx, dy):
        """
        Berechnet die Bewegungsrichtung aus Positionsänderung (dx, dy).
        
        Verwendet die Änderung der x/y Position um zu bestimmen, ob der Roboter
        vorwärts, rückwärts, links oder rechts fährt.
        
        Koordinatensystem (Roboter-Frame):
        - x positiv = vorwärts
        - x negativ = rückwärts  
        - y positiv = links
        - y negativ = rechts
        
        Args:
            dx: Änderung der x-Position
            dy: Änderung der y-Position
            
        Returns:
            Richtungs-Konstante (DIRECTION_FORWARD, DIRECTION_BACKWARD, etc.)
        """
        # Betrag der Bewegung berechnen
        magnitude = math.sqrt(dx * dx + dy * dy)
        
        if magnitude < self.POSITION_CHANGE_THRESHOLD:
            # Zu kleine Bewegung, ignorieren
            return None
        
        # Winkel der Bewegung berechnen (in Grad, 0° = vorwärts)
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        
        # Winkel normalisieren auf -180 bis 180
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        
        # Richtung basierend auf Winkel bestimmen
        # Vorwärts: -45° bis 45°
        # Links: 45° bis 135°
        # Rückwärts: 135° bis 180° oder -180° bis -135°
        # Rechts: -135° bis -45°
        
        tolerance = self.DIRECTION_ANGLE_TOLERANCE
        
        if -tolerance <= angle_deg <= tolerance:
            return DIRECTION_FORWARD
        elif tolerance < angle_deg <= (180 - tolerance):
            return DIRECTION_LEFT
        elif -(180 - tolerance) <= angle_deg < -tolerance:
            return DIRECTION_RIGHT
        else:
            return DIRECTION_BACKWARD
    
    def on_target_pose(self, msg):
        """
        Callback für /navbot/target_pose - Neues Navigationsziel (movement_api/TargetPose).
        
        TargetPose Message Struktur:
        - header: Standard ROS Header
        - target_id: string (Ziel-ID, z.B. Raumname)
        - x: float32 (X-Koordinate)
        - y: float32 (Y-Koordinate)
        - yaw: float32 (Orientierung)
        
        Wird aufgerufen wenn ein neues Ziel gesetzt wird.
        Triggert START_MOVE State.
        """
        target_id = msg.target_id if msg.target_id else ""
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Received target_pose: target_id='{target_id}', x={msg.x:.2f}, y={msg.y:.2f}")
        self.trigger_state('trigger_start_move', f"Ziel: {target_id} ({msg.x:.2f}, {msg.y:.2f})")
    
    def on_target_pose_fallback(self, msg):
        """
        Fallback-Callback für /navbot/target_pose wenn TargetPose nicht verfügbar.
        Verarbeitet geometry_msgs/PoseStamped.
        
        Args:
            msg: PoseStamped Message
        """
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Received target_pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.trigger_state('trigger_start_move', f"Ziel: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
    
    def on_nav_status(self, msg):
        """
        Callback für /navbot/nav_status - Navigationsstatus vom Movement Team.
        
        NavStatus Message Struktur (movement_api/NavStatus):
        - header: Standard ROS Header
        - state: uint8 (Konstanten: READY=0, MOVING_TO_TARGET=1, ARRIVED=2, 
                        WAITING_FOR_SPEECH=3, RETURNING_TO_START=4, AT_START=5,
                        EMERGENCY_STOP=6, FAILED=7)
        - target_id: String (Ziel-ID)
        - detail: String (zusätzliche Details)
        
        Args:
            msg: NavStatus Message (movement_api.msg)
        """
        state = msg.state
        target_id = msg.target_id if msg.target_id else ""
        detail = msg.detail if msg.detail else ""
        
        # State-Namen für Logging
        state_names = {
            NavStatusConstants.READY: "READY",
            NavStatusConstants.MOVING_TO_TARGET: "MOVING_TO_TARGET",
            NavStatusConstants.ARRIVED: "ARRIVED",
            NavStatusConstants.WAITING_FOR_SPEECH: "WAITING_FOR_SPEECH",
            NavStatusConstants.RETURNING_TO_START: "RETURNING_TO_START",
            NavStatusConstants.AT_START: "AT_START",
            NavStatusConstants.EMERGENCY_STOP: "EMERGENCY_STOP",
            NavStatusConstants.FAILED: "FAILED"
        }
        state_name = state_names.get(state, f"UNKNOWN({state})")
        rospy.loginfo(f"[SIGNAL_CONTROLLER] NavStatus: state={state_name}, target='{target_id}', detail='{detail}'")
        
        # State-basierte Verarbeitung mit uint8 Konstanten
        if state == NavStatusConstants.ARRIVED:
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Movement: GOAL REACHED (target: {target_id})")
            self.trigger_state('trigger_goal_reached', f"Ziel erreicht: {target_id}")
            return
        
        if state == NavStatusConstants.MOVING_TO_TARGET:
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Movement: Navigation gestartet")
            self.trigger_state('trigger_start_move', f"Navigation zu: {target_id}")
            return
        
        if state == NavStatusConstants.RETURNING_TO_START:
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Movement: Rückfahrt zum Start")
            self.trigger_state('trigger_start_move', "Rückfahrt zum Start")
            return
        
        if state == NavStatusConstants.READY or state == NavStatusConstants.AT_START:
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Movement: Bereit/Am Start -> IDLE")
            self.trigger_state('trigger_idle')
            return
        
        if state == NavStatusConstants.WAITING_FOR_SPEECH:
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Movement: Warte auf Spracheingabe")
            self.trigger_state('trigger_waiting', "Warte auf Spracheingabe")
            return
        
        if state == NavStatusConstants.EMERGENCY_STOP:
            rospy.logwarn("[SIGNAL_CONTROLLER] Movement: EMERGENCY STOP!")
            self._emergency_stop_active = True
            self.trigger_state('trigger_error_major', "Emergency Stop via NavStatus")
            return
        
        if state == NavStatusConstants.FAILED:
            rospy.loginfo(f"[SIGNAL_CONTROLLER] Movement: Fehler - {detail}")
            if 'stuck' in detail.lower():
                self.trigger_state('trigger_error_minor_stuck')
            else:
                self.trigger_state('trigger_error_minor_nav')
            return
    
    def on_nav_status_string(self, msg):
        """
        Fallback-Callback für /navbot/nav_status wenn NavStatus nicht verfügbar.
        Verarbeitet einfache String-Messages.
        
        Args:
            msg: String Message
        """
        text = msg.data.lower() if hasattr(msg, 'data') else str(msg).lower()
        rospy.loginfo(f"[SIGNAL_CONTROLLER] NavStatus (string): '{text}'")
        
        # GOAL_REACHED erkennen
        if 'arrived' in text or 'goal_reached' in text or 'reached' in text:
            rospy.loginfo("[SIGNAL_CONTROLLER] Movement: GOAL REACHED")
            self.trigger_state('trigger_goal_reached')
        
        # Navigation gestartet
        elif 'moving' in text or 'navigating' in text or 'started' in text:
            self.trigger_state('trigger_start_move')
        
        # Navigation gestoppt
        elif 'stopped' in text or 'aborted' in text:
            self.trigger_state('trigger_stop_move')
        
        # Fehler
        elif 'error' in text or 'failed' in text:
            self.trigger_state('trigger_error_minor_nav')
    
    def on_cmd_vel(self, msg):
        """
        Callback für /cmd_vel - Aktuelle Geschwindigkeitskommandos.
        
        Interpretiert die Geschwindigkeit um die Bewegungsrichtung zu bestimmen.
        Dies ist eine Alternative zur Positionsänderungs-Methode.
        
        Args:
            msg: Twist message mit linear.x (vorwärts/rückwärts) und angular.z (Drehung)
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Schwellenwerte
        LINEAR_THRESHOLD = 0.05
        ANGULAR_THRESHOLD = 0.3
        
        # Prüfen ob Bewegung stattfindet
        if abs(linear_x) > LINEAR_THRESHOLD or abs(angular_z) > ANGULAR_THRESHOLD:
            # Richtung bestimmen
            if linear_x < -LINEAR_THRESHOLD:
                direction = DIRECTION_BACKWARD
            elif abs(angular_z) > ANGULAR_THRESHOLD:
                direction = DIRECTION_LEFT if angular_z > 0 else DIRECTION_RIGHT
            else:
                direction = DIRECTION_FORWARD
            
            # DEBOUNCING: Nur triggern wenn Richtung anders oder Zeit abgelaufen
            now = rospy.Time.now()
            time_since_last = (now - self._last_direction_time).to_sec()
            
            if direction != self._last_direction or time_since_last > self.DIRECTION_DEBOUNCE_SEC:
                self._last_direction = direction
                self._last_direction_time = now
                self._is_moving = True
                
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
                
                rospy.loginfo(f"[SIGNAL_CONTROLLER] cmd_vel Direction: {direction_names.get(direction, 'UNKNOWN')}")
                
                # Entsprechenden MOVE State triggern
                trigger = direction_triggers.get(direction, 'trigger_move_forward')
                self.trigger_state(trigger)
        else:
            # Stillstand - Bewegung beendet
            if self._is_moving:
                self._is_moving = False
                self._last_direction = None
                rospy.loginfo("[SIGNAL_CONTROLLER] Movement stopped (cmd_vel)")
                self.trigger_state('trigger_stop_move')
    
    def on_odom(self, msg):
        """
        Callback für /odom - Odometrie-Daten für Positionstracking.
        
        Berechnet die Bewegungsrichtung aus der Änderung der Position.
        Dies ist die primäre Methode zur Richtungserkennung basierend auf 
        tatsächlichen Positionsänderungen.
        
        Args:
            msg: Odometry message mit aktueller Position
        """
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_time = rospy.Time.now()
        
        # Erste Position speichern
        if self._last_position_x is None:
            self._last_position_x = current_x
            self._last_position_y = current_y
            self._last_position_time = current_time
            return
        
        # Positionsänderung berechnen
        dx = current_x - self._last_position_x
        dy = current_y - self._last_position_y
        
        # Richtung aus Positionsänderung berechnen
        direction = self.calculate_direction_from_position_change(dx, dy)
        
        if direction is not None:
            # Signifikante Bewegung erkannt
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
                
                rospy.loginfo(f"[SIGNAL_CONTROLLER] Odom Direction: {direction_names.get(direction, 'UNKNOWN')} "
                             f"(dx={dx:.3f}, dy={dy:.3f})")
                
                # Entsprechenden MOVE State triggern
                trigger = direction_triggers.get(direction, 'trigger_move_forward')
                self.trigger_state(trigger)
        
        # Position für nächsten Vergleich speichern
        self._last_position_x = current_x
        self._last_position_y = current_y
        self._last_position_time = current_time
    
    def on_emergency_stop(self, msg):
        """
        Callback für /emergency_stop - Notaus (movement_api/EmergencyStop).
        
        EmergencyStop Message Struktur:
        - header: Standard ROS Header
        - active: bool (True = Notaus aktiv, False = Notaus gelöst)
        - reason: string (Grund für den Notaus)
        
        WICHTIG: Behandelt sowohl Aktivierung als auch Deaktivierung des Notaus!
        - msg.active == True:  Notaus aktiviert → ERROR_MAJOR
        - msg.active == False: Notaus gelöst → zurück zu IDLE
        """
        reason = msg.reason if msg.reason else "Unbekannt"
        
        if msg.active:
            # Notaus AKTIVIERT
            if not self._emergency_stop_active:
                rospy.logwarn(f"[SIGNAL_CONTROLLER] !! EMERGENCY STOP ACTIVATED !! Grund: {reason}")
                self._emergency_stop_active = True
                self.trigger_state('trigger_error_major', f"Emergency Stop: {reason}")
        else:
            # Notaus GELÖST
            if self._emergency_stop_active:
                rospy.loginfo("[SIGNAL_CONTROLLER] Emergency stop RELEASED - returning to IDLE")
                self._emergency_stop_active = False
                self.trigger_state('trigger_idle', "Emergency Stop released")
    
    def on_emergency_stop_bool(self, msg):
        """
        Fallback-Callback für /emergency_stop wenn EmergencyStop Message nicht verfügbar.
        Verarbeitet einfache Bool-Messages.
        
        Args:
            msg: Bool Message (True = Notaus aktiv, False = Notaus gelöst)
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
    
    def on_speech_out_speaking(self, msg):
        """
        Callback für /speech_out/is_speaking - Roboter spricht (Speech-Out Team).
        
        Wenn der Roboter spricht (Speech-Out aktiv):
        - Signal-Sounds werden auf 25% Lautstärke reduziert
        - SPEAKING State wird aktiviert
        
        Wenn Speech-Out beendet:
        - Signal-Sounds werden auf 100% Lautstärke zurückgesetzt
        
        Args:
            msg: Bool Message (True = Roboter spricht, False = fertig)
        """
        if msg.data:
            rospy.loginfo("[SIGNAL_CONTROLLER] Speech-Out: Roboter spricht -> Lautstärke reduzieren")
            set_volume_for_speaking(True)
            self.trigger_state('trigger_speaking')
        else:
            rospy.loginfo("[SIGNAL_CONTROLLER] Speech-Out: Roboter fertig -> Lautstärke normal")
            set_volume_for_speaking(False)
            self.trigger_state('trigger_stop_busy')
    
    def on_speech_in_status(self, msg):
        """
        Callback für /speech_output - SpeechStatus Message vom Speech-In Team.
        
        Speech-In = Spracherkennung (User Input), NICHT Roboter-Sprachausgabe!
        
        SpeechStatus Message Struktur:
        - header: Standard ROS Header
        - level: String (z.B. "info", "error", "warning")
        - message: String (erkannter Text oder Fehlermeldung)
        
        Verarbeitet erkannte Spracheingaben:
        - message enthält "room not found" → ROOM_NOT_FOUND State
        - message enthält "greeting/hallo" → GREETING State
        
        HINWEIS: GOAL_REACHED kommt vom Movement Team, nicht von hier!
        
        Args:
            msg: SpeechStatus Message (speech_in.msg)
        """
        level = msg.level.lower() if msg.level else ""
        message = msg.message.lower() if msg.message else ""
        
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Speech-In: level='{level}', message='{message[:50]}...'")
        
        # Prüfe message auf spezielle Inhalte
        if message:
            # Room Not Found erkennen
            if 'room not found' in message or 'raum nicht gefunden' in message or 'nicht gefunden' in message:
                rospy.loginfo("[SIGNAL_CONTROLLER] Speech-In: Room Not Found erkannt")
                self.trigger_state('trigger_room_not_found')
                return
            
            # Greeting erkennen (User sagt Hallo)
            if 'greeting' in message or 'hello' in message or 'hallo' in message or 'guten tag' in message:
                rospy.loginfo("[SIGNAL_CONTROLLER] Speech-In: Greeting erkannt")
                self.trigger_state('trigger_greeting')
                return
    
    def on_speech_in_status_string(self, msg):
        """
        Fallback-Callback für /speech_output wenn SpeechStatus nicht verfügbar.
        Verarbeitet einfache String-Messages vom Speech-In Team.
        
        HINWEIS: GOAL_REACHED kommt vom Movement Team, nicht von hier!
        
        Args:
            msg: String Message
        """
        text = msg.data.lower() if hasattr(msg, 'data') else str(msg).lower()
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Speech-In (string): '{text[:50]}...'")
        
        # Room Not Found erkennen
        if 'room not found' in text or 'raum nicht gefunden' in text or 'nicht gefunden' in text:
            rospy.loginfo("[SIGNAL_CONTROLLER] Speech-In: Room Not Found erkannt")
            self.trigger_state('trigger_room_not_found')
        
        # Greeting erkennen
        elif 'greeting' in text or 'hello' in text or 'hallo' in text or 'guten tag' in text:
            rospy.loginfo("[SIGNAL_CONTROLLER] Speech-In: Greeting erkannt")
            self.trigger_state('trigger_greeting')
    
    def on_navigation_error(self, msg):
        """Callback für /directions/navigation_error - Navigationsfehler."""
        error = msg.data.lower()
        rospy.loginfo(f"[SIGNAL_CONTROLLER] Received: navigation_error ({error})")
        
        if 'room_not_found' in error or 'nicht gefunden' in error or 'not found' in error:
            self.trigger_state('trigger_room_not_found')
        else:
            self.trigger_state('trigger_error_minor_nav')
    
    def on_battery_state(self, msg):
        """Callback für monitoring/battery_state - Batteriestatus."""
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
