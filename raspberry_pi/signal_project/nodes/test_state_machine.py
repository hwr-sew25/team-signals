#!/usr/bin/env python3
"""
Interaktives Test-Script für die SMACH State Machine.
Kann mit rosrun signal_project test_state_machine.py gestartet werden.
"""

import rospy
import smach_ros
import threading

from signal_project.state_machine.state_machine import create_state_machine, get_idle_state
from signal_project.state_machine.signal_state_defs import SignalState
from signal_project.state_machine.states.move_state import MoveState
from signal_project.led_engine.led_engine import (
    send_move_direction,
    DIRECTION_FORWARD,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    DIRECTION_BACKWARD
)

# Globale Referenz zur State Machine für preemption
_state_machine = None


# Mapping von Benutzer-Eingabe zu Trigger
STATE_TRIGGERS = {
    'GREETING': 'trigger_greeting',
    'IDLE': 'trigger_idle',
    'BUSY': 'trigger_busy',
    'STOP_BUSY': 'trigger_stop_busy',
    'ERROR_MINOR_STUCK': 'trigger_error_minor_stuck',
    'ERROR_MINOR_NAV': 'trigger_error_minor_nav',
    'ROOM_NOT_FOUND': 'trigger_room_not_found',
    'ERROR_MAJOR': 'trigger_error_major',
    'LOW_BATTERY': 'trigger_low_battery',
    'MOVE': 'trigger_move',
    'START_MOVE': 'trigger_start_move',
    'STOP_MOVE': 'trigger_stop_move',
    'GOAL_REACHED': 'trigger_goal_reached',
    'REVERSE': 'trigger_reverse',
    'SPEAKING': 'trigger_speaking',
    'WAITING': 'trigger_waiting',
}

# Spezielle Richtungs-Commands für MOVE State
MOVE_DIRECTIONS = {
    'MOVE_LEFT': DIRECTION_LEFT,
    'MOVE_FORWARD': DIRECTION_FORWARD,
    'MOVE_RIGHT': DIRECTION_RIGHT,
    'MOVE_BACKWARD': DIRECTION_BACKWARD,
}


def trigger_new_state(idle_state, trigger):
    """
    Triggert einen neuen State.
    Preempted zuerst den aktuellen State, dann setzt den neuen Trigger.
    """
    global _state_machine
    
    if idle_state is None:
        print("[ERROR] IDLE State nicht verfügbar")
        return
    
    # Zuerst den Trigger setzen (damit IDLE ihn sofort sieht)
    idle_state.set_trigger(trigger)
    
    # Dann den aktuellen State preempten
    if _state_machine is not None:
        _state_machine.request_preempt()


def input_thread(idle_state):
    """Thread für interaktive Benutzereingaben."""
    print("\n" + "="*60)
    print("           INTERAKTIVER STATE TRIGGER")
    print("="*60)
    print("\nVerfügbare States:")
    print("-" * 60)
    print("  GREETING           - Begrüßung")
    print("  IDLE               - Ruhezustand")
    print("  BUSY               - Roboter beschäftigt")
    print("  STOP_BUSY          - Busy beenden")
    print("  ERROR_MINOR_STUCK  - Stuck (steckengeblieben)")
    print("  ERROR_MINOR_NAV    - Navigation Error")
    print("  ROOM_NOT_FOUND     - Raum existiert nicht")
    print("  ERROR_MAJOR        - Schwerer Fehler / Emergency")
    print("  LOW_BATTERY        - Niedriger Akkustand")
    print("-" * 60)
    print("  MOVE               - Bewegung (Standardrichtung)")
    print("  MOVE_LEFT          - Bewegung Links (LEDs 0-15)")
    print("  MOVE_FORWARD       - Bewegung Vorwärts (LEDs 16-31)")
    print("  MOVE_RIGHT         - Bewegung Rechts (LEDs 32-47)")
    print("  MOVE_BACKWARD      - Bewegung Rückwärts (LEDs 48-63)")
    print("-" * 60)
    print("  START_MOVE         - Bewegung startet")
    print("  STOP_MOVE          - Bewegung stoppt")
    print("  GOAL_REACHED       - Ziel erreicht")
    print("  REVERSE            - Rückwärtsfahrt")
    print("  SPEAKING           - Sprachausgabe (LED, kein Sound)")
    print("  WAITING            - Warten (Lichtwelle)")
    print("-" * 60)
    print("  QUIT               - Beenden")
    print("="*60 + "\n")
    
    while not rospy.is_shutdown():
        try:
            user_input = input("State > ").strip().upper()
            
            if user_input == 'QUIT' or user_input == 'Q':
                rospy.loginfo("[INPUT] Shutdown requested")
                rospy.signal_shutdown("User requested shutdown")
                break
            
            if user_input == '':
                continue
            
            # Spezielle Richtungs-Commands für MOVE State
            if user_input in MOVE_DIRECTIONS:
                direction = MOVE_DIRECTIONS[user_input]
                rospy.loginfo(f"[INPUT] Setting direction to {user_input}")
                
                # MoveState Richtung setzen
                MoveState.set_direction(direction)
                
                # Neuen State triggern (preempted aktuellen State)
                trigger_new_state(idle_state, 'trigger_move')
                continue
            
            # Prüfe ob State existiert
            if user_input in STATE_TRIGGERS:
                trigger = STATE_TRIGGERS[user_input]
                rospy.loginfo(f"[INPUT] Triggering {user_input} state")
                
                # Neuen State triggern (preempted aktuellen State)
                trigger_new_state(idle_state, trigger)
            else:
                print(f"[WARN] Unbekannter State: {user_input}")
                print("Verfügbar:", ", ".join(list(STATE_TRIGGERS.keys()) + list(MOVE_DIRECTIONS.keys())))
                
        except EOFError:
            break
        except Exception as e:
            rospy.logwarn(f"[INPUT] Error: {e}")
            break


def main():
    """Test der State Machine mit interaktivem Trigger."""
    global _state_machine
    
    print("="*55)
    print("    TEAM SIGNALE — SMACH STATE MACHINE TEST")
    print("="*55)
    
    # ROS Node initialisieren
    rospy.init_node('test_state_machine', anonymous=False)
    rospy.loginfo("[TEST] Starting State Machine Test")
    
    # State Machine erstellen
    sm = create_state_machine()
    _state_machine = sm  # Globale Referenz für preemption
    rospy.loginfo("[TEST] State Machine created with all states")
    
    # Introspection Server für Debugging
    sis = smach_ros.IntrospectionServer('test_sm_server', sm, '/TEST_SM')
    sis.start()
    
    # IDLE State für Trigger holen
    idle_state = get_idle_state(sm)
    
    if idle_state is None:
        rospy.logerr("[TEST] IDLE State not found!")
        return
    
    # Input-Thread starten
    input_t = threading.Thread(target=input_thread, args=(idle_state,), daemon=True)
    input_t.start()
    
    try:
        # State Machine ausführen
        rospy.loginfo("[TEST] State Machine running - waiting for input...")
        outcome = sm.execute()
        rospy.loginfo(f"[TEST] State Machine finished with outcome: {outcome}")
    except rospy.ROSInterruptException:
        rospy.loginfo("[TEST] ROS Interrupt received")
    finally:
        sis.stop()
        rospy.loginfo("[TEST] Test completed")


if __name__ == "__main__":
    main()
