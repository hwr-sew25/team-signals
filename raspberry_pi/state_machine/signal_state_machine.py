import serial
from audio_engine import play_sound
from signal_state_defs import SignalState

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def send_state(state_id):
    ser.write(f"{state_id}\n".encode())

def greeting():
    send_state(0)
    play_sound("greeting")

def error_major():
    send_state(4)
    play_sound("error")

def reverse():
    send_state(10)
    play_sound("reverse")

