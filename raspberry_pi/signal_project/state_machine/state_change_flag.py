#!/usr/bin/env python3
"""
Shared State Change Flag für die State Machine.

Dieses Modul enthält ein globales Flag, das signalisiert, dass ein
Zustandswechsel angefordert wurde. Alle States prüfen dieses Flag
in ihren while-Schleifen, um schnell auf Zustandswechsel zu reagieren
und Deadlocks zu vermeiden.
"""

import threading

# Thread-sicheres Lock für den Zugriff auf das Flag
_lock = threading.Lock()

# Flag das signalisiert, dass ein Zustandswechsel angefordert wurde
_state_change_requested = False


def request_state_change():
    """
    Signalisiert, dass ein Zustandswechsel angefordert wurde.
    Wird vom SignalControllerNode aufgerufen.
    """
    global _state_change_requested
    with _lock:
        _state_change_requested = True


def is_state_change_requested():
    """
    Prüft, ob ein Zustandswechsel angefordert wurde.
    Wird von States in ihren while-Schleifen aufgerufen.
    
    Returns:
        bool: True wenn ein Zustandswechsel angefordert wurde
    """
    with _lock:
        return _state_change_requested


def clear_state_change_request():
    """
    Setzt das Flag zurück.
    Wird vom IDLE State aufgerufen, nachdem der Zustandswechsel verarbeitet wurde.
    """
    global _state_change_requested
    with _lock:
        _state_change_requested = False

