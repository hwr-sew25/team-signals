#!/usr/bin/env python3
"""
LED Engine für die Kommunikation mit dem Arduino LED-Controller.
"""

import serial
import time

DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]

_serial_connection = None


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

