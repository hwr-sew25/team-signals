import os
import serial
from audio_engine.play_sound import play_sound
from signal_state_defs import SignalState

# Serial Verbindung zum Arduino
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Sound-Pfad
SOUND_DIR = "/home/ubuntu/team-signals/raspberry_pi/audio_engine/sounds/"

# Mapping: STATE → WAV-Dateiname
STATE_TO_SOUND = {
    SignalState.GREETING: "greeting.wav",
    SignalState.IDLE: None,
    SignalState.BUSY: None,
    SignalState.ERROR_MINOR: "error_minor.wav",
    SignalState.ERROR_MAJOR: "error.wav",
    SignalState.LOW_BATTERY: "lowbattery.wav",
    SignalState.MOVE: None,
    SignalState.START_MOVE: "start_move.wav",
    SignalState.STOP_MOVE: "stop_move.wav",
    SignalState.REVERSE: "reverse.wav",
    SignalState.SPEAKING: "speaking.wav",
}

def send_state(state: SignalState):
    """Sendet StateID an den Arduino."""
    ser.write(f"{state.value}\n".encode())

def play_state_sound(state: SignalState):
    """Spielt passende WAV-Datei ab."""
    filename = STATE_TO_SOUND.get(state)
    if not filename:
        return
    path = os.path.join(SOUND_DIR, filename)
    play_sound(path)

def trigger_state(state: SignalState):
    """Öffentliches Interface: LED + Sound."""
    print(f"[STATE MACHINE] Trigger → {state.name}")
    send_state(state)
    play_state_sound(state)

