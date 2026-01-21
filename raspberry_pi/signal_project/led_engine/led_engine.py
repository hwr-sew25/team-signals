#!/usr/bin/env python3
"""
LED Engine für die Kommunikation mit dem Arduino LED-Controller.

WICHTIG: 
- Enthält automatischen Reconnect bei Verbindungsabbruch
- Thread-safe durch konsequentes Locking
- Graceful Shutdown mit cleanup_connection()
"""

import serial
import time
import threading

DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]

_serial_connection = None
_serial_lock = threading.Lock()  # Thread-Safety für Serial-Zugriff

# Richtungs-Konstanten für MOVE State
# Entspricht den Segmenten auf dem LED-Strip (64 LEDs, 4x16)
DIRECTION_BACKWARD = 0  # Segment 0: LEDs 0-15  (Rückwärts)
DIRECTION_RIGHT = 1     # Segment 1: LEDs 16-31 (Rechts)
DIRECTION_FORWARD = 2   # Segment 2: LEDs 32-47 (Vorwärts)
DIRECTION_LEFT = 3      # Segment 3: LEDs 48-63 (Links)

# Mapping von Richtungs-Konstanten zu Arduino-Commands
DIRECTION_COMMANDS = {
    DIRECTION_LEFT: "MOVE_LEFT",
    DIRECTION_FORWARD: "MOVE_FORWARD",
    DIRECTION_RIGHT: "MOVE_RIGHT",
    DIRECTION_BACKWARD: "MOVE_BACKWARD",
}


def _detect_port_internal():
    """
    Interne Funktion: Erkennt den Arduino-Port.
    HINWEIS: Wird ohne Lock aufgerufen - Lock muss vom Aufrufer gehalten werden!
    """
    for port in DEFAULT_PORTS:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)  # Arduino reset nach Serial-Open
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print(f"[LED_ENGINE] Arduino found on {port}")
            return ser
        except Exception as e:
            print(f"[LED_ENGINE] Port {port} not usable: {e}")
    print("[LED_ENGINE] No Arduino found!")
    return None


def detect_port():
    """
    Öffentliche Funktion: Erkennt den Arduino-Port (thread-safe).
    """
    with _serial_lock:
        return _detect_port_internal()


def get_serial_connection():
    """
    Lazy-Initialisierung der Serial-Verbindung (thread-safe).
    """
    global _serial_connection
    with _serial_lock:
        if _serial_connection is None:
            _serial_connection = _detect_port_internal()
        return _serial_connection


def cleanup_connection():
    """
    Schließt die Serial-Verbindung sauber.
    Sollte beim Shutdown aufgerufen werden.
    """
    global _serial_connection
    with _serial_lock:
        if _serial_connection is not None:
            try:
                _serial_connection.close()
                print("[LED_ENGINE] Serial connection closed cleanly")
            except Exception as e:
                print(f"[LED_ENGINE] Error closing connection: {e}")
            finally:
                _serial_connection = None


def send_led_command(state):
    """
    Sendet einen LED-Befehl an den Arduino (thread-safe).
    
    WICHTIG: 
    - Gesamte Funktion unter einem Lock für Thread-Safety
    - Bei Verbindungsfehlern wird automatisch ein Reconnect versucht
    - Wartet auf "OK" Bestätigung vom Arduino
    
    Args:
        state: SignalState enum oder String
        
    Returns:
        bool: True wenn erfolgreich, False bei Fehler
    """
    global _serial_connection
    
    # Command vorbereiten
    if hasattr(state, "name"):
        command = state.name
    else:
        command = str(state)
    
    cmd = command.strip().upper() + ";"
    
    with _serial_lock:
        # Verbindung herstellen falls nötig
        if _serial_connection is None:
            _serial_connection = _detect_port_internal()
            if _serial_connection is None:
                print("[LED_ENGINE] ERROR: Arduino not connected")
                return False
        
        print(f"[LED_ENGINE] Sending command: {cmd.strip()}")
        
        try:
            _serial_connection.write(cmd.encode("ascii", errors="ignore"))
            _serial_connection.flush()
            
            # Auf "OK" Bestätigung warten (mit Timeout)
            _serial_connection.timeout = 0.5  # 500ms Timeout
            response = _serial_connection.readline().decode("ascii", errors="ignore").strip()
            
            if response.startswith("OK"):
                print(f"[LED_ENGINE] Arduino confirmed: {response}")
                return True
            elif response:
                print(f"[LED_ENGINE] Arduino response: {response}")
                return True  # Auch andere Antworten akzeptieren
            else:
                # Kein Response ist auch OK (für Kompatibilität mit altem Arduino-Code)
                print(f"[LED_ENGINE] Sent (no response): {cmd.strip()}")
                return True
                
        except Exception as e:
            print(f"[LED_ENGINE] Error sending command: {e}")
            # Bei Fehler: Verbindung zurücksetzen für automatischen Reconnect
            try:
                _serial_connection.close()
            except:
                pass
            _serial_connection = None
            return False


def send_move_direction(direction):
    """
    Sendet einen Richtungs-Befehl für den MOVE State (thread-safe).
    
    Args:
        direction: Richtung (DIRECTION_LEFT, DIRECTION_FORWARD, 
                            DIRECTION_RIGHT, DIRECTION_BACKWARD)
                            
    Returns:
        bool: True wenn erfolgreich, False bei Fehler
    """
    global _serial_connection
    
    if direction not in DIRECTION_COMMANDS:
        print(f"[LED_ENGINE] ERROR: Invalid direction {direction}")
        return False
    
    command = DIRECTION_COMMANDS[direction]
    cmd = command + ";"
    
    with _serial_lock:
        # Verbindung herstellen falls nötig
        if _serial_connection is None:
            _serial_connection = _detect_port_internal()
            if _serial_connection is None:
                print("[LED_ENGINE] ERROR: Arduino not connected")
                return False
        
        print(f"[LED_ENGINE] Sending direction: {cmd.strip()}")
        
        try:
            _serial_connection.write(cmd.encode("ascii", errors="ignore"))
            _serial_connection.flush()
            
            # Auf "OK" Bestätigung warten (mit Timeout)
            _serial_connection.timeout = 0.5
            response = _serial_connection.readline().decode("ascii", errors="ignore").strip()
            
            if response.startswith("OK"):
                print(f"[LED_ENGINE] Arduino confirmed: {response}")
            elif response:
                print(f"[LED_ENGINE] Arduino response: {response}")
                
            return True
            
        except Exception as e:
            print(f"[LED_ENGINE] Error sending direction: {e}")
            # Bei Fehler: Verbindung zurücksetzen
            try:
                _serial_connection.close()
            except:
                pass
            _serial_connection = None
            return False


def calculate_direction_from_twist(linear_x, angular_z):
    """
    Berechnet die Fahrtrichtung aus Twist-Daten (linear.x, angular.z).
    
    Args:
        linear_x: Vorwärts/Rückwärts Geschwindigkeit
        angular_z: Drehgeschwindigkeit (positiv = links, negativ = rechts)
    
    Returns:
        Richtungs-Konstante (DIRECTION_LEFT, DIRECTION_FORWARD, etc.)
    """
    # Schwellenwerte für die Richtungserkennung
    LINEAR_THRESHOLD = 0.05
    ANGULAR_THRESHOLD = 0.3
    
    # Rückwärts?
    if linear_x < -LINEAR_THRESHOLD:
        return DIRECTION_BACKWARD
    
    # Hauptsächlich Drehung?
    if abs(angular_z) > ANGULAR_THRESHOLD:
        if angular_z > 0:
            return DIRECTION_LEFT
        else:
            return DIRECTION_RIGHT
    
    # Vorwärts (oder minimal)
    if linear_x > LINEAR_THRESHOLD:
        return DIRECTION_FORWARD
    
    # Default: Vorwärts
    return DIRECTION_FORWARD
