import serial
import time

DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]

def detect_port():
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

SER = detect_port()

def send_led_command(state):
    if SER is None:
        print("[LED_ENGINE] ERROR: Arduino not connected")
        return

    if hasattr(state, "name"):
        command = state.name
    else:
        command = str(state)

    cmd = command.strip().upper() + ";"
    print(f"[LED_ENGINE] ABOUT TO SEND repr={repr(cmd)} len={len(cmd)}")

    SER.write(cmd.encode("ascii", errors="ignore"))
    SER.flush()
    print(f"[LED_ENGINE] Sent LED command: {cmd.strip()}")


