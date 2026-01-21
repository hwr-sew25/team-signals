#!/usr/bin/env python3
"""
LED Engine für die Kommunikation mit dem Arduino LED-Controller.
"""

import serial
import time

DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]

_serial_connection = None

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


def detect_port():
    """Erkennt den Arduino-Port und gibt die Serial-Verbindung zurück."""
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


def get_serial_connection():
    """Lazy-Initialisierung der Serial-Verbindung."""
    global _serial_connection
    if _serial_connection is None:
        _serial_connection = detect_port()
    return _serial_connection


def send_led_command(state):
    """Sendet einen LED-Befehl an den Arduino."""
    ser = get_serial_connection()
    
    if ser is None:
        print("[LED_ENGINE] ERROR: Arduino not connected")
        return False

    if hasattr(state, "name"):
        command = state.name
    else:
        command = str(state)

    cmd = command.strip().upper() + ";"
    print(f"[LED_ENGINE] Sending command: {cmd.strip()}")

    try:
        ser.write(cmd.encode("ascii", errors="ignore"))
        ser.flush()
        print(f"[LED_ENGINE] Sent LED command: {cmd.strip()}")
        return True
    except Exception as e:
        print(f"[LED_ENGINE] Error sending command: {e}")
        return False


def send_move_direction(direction):
    """
    Sendet einen Richtungs-Befehl für den MOVE State.
    
    Args:
        direction: Richtung (DIRECTION_LEFT, DIRECTION_FORWARD, 
                            DIRECTION_RIGHT, DIRECTION_BACKWARD)
    """
    ser = get_serial_connection()
    
    if ser is None:
        print("[LED_ENGINE] ERROR: Arduino not connected")
        return False
    
    if direction not in DIRECTION_COMMANDS:
        print(f"[LED_ENGINE] ERROR: Invalid direction {direction}")
        return False
    
    command = DIRECTION_COMMANDS[direction]
    cmd = command + ";"
    print(f"[LED_ENGINE] Sending direction command: {cmd.strip()}")
    
    try:
        ser.write(cmd.encode("ascii", errors="ignore"))
        ser.flush()
        print(f"[LED_ENGINE] Sent direction command: {cmd.strip()}")
        return True
    except Exception as e:
        print(f"[LED_ENGINE] Error sending direction command: {e}")
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

