#!/usr/bin/env python3
"""
Audio Engine für Sound-Wiedergabe.
"""

import os
import subprocess

# Absoluter Pfad zum Sound-Ordner
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SOUND_DIR = os.path.join(BASE_DIR, "sounds")


def play_sound(filename):
    """
    Spielt eine WAV-Datei mit aplay über den WM8960 (plughw:2,0).
    
    Args:
        filename: Dateiname der WAV-Datei (z.B. "greeting.wav")
    """
    if not filename:
        return False
        
    path = os.path.join(SOUND_DIR, filename)
    
    if not os.path.exists(path):
        print(f"[AUDIO_ENGINE] Sound not found: {path}")
        return False

    print(f"[AUDIO_ENGINE] Playing: {path}")
    try:
        subprocess.call(["aplay", "-D", "plughw:2,0", path])
        return True
    except Exception as e:
        print(f"[AUDIO_ENGINE] Error playing sound: {e}")
        return False


def play_state_sound(filename):
    """
    Spielt einen State-spezifischen Sound ab.
    
    Args:
        filename: Dateiname der WAV-Datei (z.B. "greeting.wav")
    """
    if not filename:
        return False
    return play_sound(filename)
