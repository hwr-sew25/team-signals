import os
import serial
from signal_project.audio_engine.audio_engine import play_state_sound
from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.led_engine.led_engine import send_led_command

# Absoluter Pfad zum Sound-Ordner
BASE_DIR= os.path.dirname(
	os.path.dirname(os.path.abspath(__file__))
)

SOUND_DIR = os.path.join(BASE_DIR, "audio_engine", "sounds")
	
# Serial Verbindung zum Arduino
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

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

def trigger_state(state: SignalState):
    print(f"[STATE MACHINE] Trigger → {state.name}")

    # LED zuerst (sichtbare Reaktion)
    send_led_command(state)

    # optionaler Sound
    filename = STATE_TO_SOUND.get(state)
    if filename:
        play_state_sound(filename)

