#!/usr/bin/env python3
"""
Legacy-Modul - wird für Rückwärtskompatibilität beibehalten.
Bitte verwende signal_project.led_engine stattdessen.
"""

# Re-export für Rückwärtskompatibilität
from signal_project.led_engine.led_engine import send_led_command, detect_port, get_serial_connection
