import serial
import time

# Serieller Port – wird automatisch gefunden
DEFAULT_PORTS = ['/dev/ttyACM0', '/dev/ttyUSB0']

def detect_port():
    for port in DEFAULT_PORTS:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            time.sleep(1)
            print(f"[LED_ENGINE] Connected to {port}")
            return ser
        except:
            pass
    print("[LED_ENGINE] No Arduino found!")
    return None


SER = detect_port()


def send_led_command(command: str):
    """
    Sendet ein LED-Pattern an den Arduino.
    Muss mit Kommando-Parser im Arduino übereinstimmen.
    """
    if SER is None:
        print("[LED_ENGINE] ERROR: Arduino not connected")
        return

    cmd = command.strip().upper() + "\n"
    SER.write(cmd.encode())
    print(f"[LED_ENGINE] Sent LED command: {cmd.strip()}")

