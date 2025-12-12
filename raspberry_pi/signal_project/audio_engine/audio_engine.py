import os
import subprocess

# Absoluter Pfad zu deinen WAV-Dateien
SOUND_DIR = "/home/ubuntu/team-signals/raspberry_pi/audio_engine/sounds"

def play_sound(name: str):
    """
    Spielt eine WAV-Datei mit aplay über den WM8960 (plughw:2,0).
    name = Basisname ohne .wav, z.B. "greeting"
    """
    path = os.path.join(SOUND_DIR, f"{name}")
    if not os.path.exists(path):
        print("[AUDIO_ENGINE] Sound not found:", path)
        return

    print("[AUDIO_ENGINE] Playing:", path)
    subprocess.call(["aplay", "-D", "plughw:2,0", path])

def play_state_sound(filename: str):
    if not filename:
        return
    path = os.path.join(SOUND_DIR, filename)
    play_sound(path)
