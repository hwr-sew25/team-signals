#!/usr/bin/env python3
"""
Test-Skript für den Arduino LED Controller.

Sendet serielle Kommandos an den Arduino und testet die Prioritäts-Logik.

Verwendung:
    python send_state.py [COM_PORT] [COMMAND]
    
Beispiele:
    python send_state.py COM3 IDLE
    python send_state.py COM3 ERROR_MAJOR
    python send_state.py COM3 FORCE_IDLE
    python send_state.py COM3 RESET
    python send_state.py COM3 STATUS
    python send_state.py COM3 TEST_PRIORITY
"""

import sys
import time
import serial
import serial.tools.list_ports


def list_ports():
    """Listet alle verfügbaren seriellen Ports auf."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("Keine seriellen Ports gefunden!")
        return []
    
    print("Verfügbare Ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")
    return ports


def send_command(ser: serial.Serial, command: str, wait_response: bool = True) -> str:
    """Sendet ein Kommando an den Arduino und wartet auf Antwort."""
    # Command mit Terminator senden
    cmd = f"{command};"
    ser.write(cmd.encode('utf-8'))
    print(f">>> {command}")
    
    if not wait_response:
        return ""
    
    # Auf Antwort warten (mit Timeout)
    time.sleep(0.1)
    response_lines = []
    timeout = time.time() + 1.0  # 1 Sekunde Timeout
    
    while time.time() < timeout:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                response_lines.append(line)
                print(f"<<< {line}")
        else:
            if response_lines:
                break
            time.sleep(0.05)
    
    return "\n".join(response_lines)


def test_priority_system(ser: serial.Serial):
    """
    Testet das Prioritäts-System automatisch.
    
    Test-Szenario:
    1. IDLE setzen (Basis)
    2. ERROR_MAJOR setzen (P0 - sollte überschreiben)
    3. IDLE versuchen (P2 - sollte BLOCKIERT werden)
    4. FORCE_IDLE verwenden (sollte überschreiben)
    5. MOVE_FORWARD setzen (P1)
    6. SPEAKING versuchen (P1 - gleiche Priorität, sollte überschreiben)
    7. LOW_BATTERY setzen (P0 - sollte überschreiben)
    8. RESET verwenden
    """
    print("\n" + "=" * 60)
    print("PRIORITÄTS-SYSTEM TEST")
    print("=" * 60 + "\n")
    
    tests = [
        ("RESET", "Basis-Reset"),
        ("STATUS", "Status prüfen (sollte IDLE sein)"),
        ("ERROR_MAJOR", "P0 State setzen"),
        ("IDLE", "P2 versuchen (sollte BLOCKIERT werden!)"),
        ("STATUS", "Status prüfen (sollte ERROR_MAJOR sein)"),
        ("FORCE_IDLE", "Force Override verwenden"),
        ("STATUS", "Status prüfen (sollte IDLE sein)"),
        ("MOVE_FORWARD", "P1 State setzen"),
        ("SPEAKING", "P1 versuchen (gleiche Priorität - sollte OK sein)"),
        ("STATUS", "Status prüfen (sollte SPEAKING sein)"),
        ("LOW_BATTERY", "P0 State setzen (sollte überschreiben)"),
        ("STATUS", "Status prüfen (sollte LOW_BATTERY sein)"),
        ("GREETING", "P1 versuchen (sollte BLOCKIERT werden!)"),
        ("STATUS", "Status prüfen (sollte LOW_BATTERY sein)"),
        ("STOP_MOVE", "Sicherheits-State (immer erlaubt)"),
        ("STATUS", "Status prüfen (sollte STOP_MOVE sein)"),
        ("RESET", "Finaler Reset"),
    ]
    
    results = []
    for cmd, description in tests:
        print(f"\n--- {description} ---")
        response = send_command(ser, cmd)
        results.append((cmd, description, response))
        time.sleep(0.3)  # Kurze Pause zwischen Tests
    
    print("\n" + "=" * 60)
    print("TEST ZUSAMMENFASSUNG")
    print("=" * 60)
    
    for cmd, desc, resp in results:
        status = "✓" if resp else "?"
        print(f"{status} {cmd}: {desc}")
        if resp:
            for line in resp.split("\n"):
                print(f"   -> {line}")
    
    print("\n" + "=" * 60)
    print("ERWARTETES VERHALTEN:")
    print("- IDLE nach ERROR_MAJOR: BLOCKED")
    print("- GREETING nach LOW_BATTERY: BLOCKED")
    print("- FORCE_* Prefix: Immer erlaubt")
    print("- STOP_MOVE: Immer erlaubt (Sicherheit)")
    print("=" * 60)


def interactive_mode(ser: serial.Serial):
    """Interaktiver Modus - Kommandos manuell eingeben."""
    print("\nInteraktiver Modus (Ctrl+C zum Beenden)")
    print("Verfügbare States:")
    print("  GREETING, IDLE")
    print("  ERROR_MINOR_STUCK, ERROR_MINOR_NAV, ROOM_NOT_FOUND, ERROR_MAJOR")
    print("  LOW_BATTERY, MOVE_LEFT, MOVE_FORWARD, MOVE_RIGHT, MOVE_BACKWARD")
    print("  START_MOVE, STOP_MOVE, GOAL_REACHED, SPEAKING, WAITING")
    print("\nSpezielle Kommandos:")
    print("  RESET        - Erzwingt IDLE")
    print("  STATUS       - Zeigt aktuellen State")
    print("  FORCE_<STATE> - Erzwingt State (z.B. FORCE_IDLE)")
    print()
    
    try:
        while True:
            cmd = input("Kommando: ").strip().upper()
            if cmd:
                send_command(ser, cmd)
    except KeyboardInterrupt:
        print("\nBeendet.")


def main():
    if len(sys.argv) < 2:
        list_ports()
        print("\nVerwendung: python send_state.py <PORT> [COMMAND]")
        print("            python send_state.py <PORT> TEST_PRIORITY")
        print("            python send_state.py <PORT>  (interaktiver Modus)")
        sys.exit(1)
    
    port = sys.argv[1]
    command = sys.argv[2].upper() if len(sys.argv) > 2 else None
    
    try:
        print(f"Verbinde mit {port}...")
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)  # Warte auf Arduino Reset
        
        # Flush input buffer
        ser.reset_input_buffer()
        
        # READY-Nachricht lesen (falls vorhanden)
        time.sleep(0.5)
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"<<< {line}")
        
        if command == "TEST_PRIORITY":
            test_priority_system(ser)
        elif command:
            send_command(ser, command)
        else:
            interactive_mode(ser)
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Fehler: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

