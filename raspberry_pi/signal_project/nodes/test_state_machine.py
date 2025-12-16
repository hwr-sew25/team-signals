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


# Mapping von Benutzer-Eingabe zu Trigger
STATE_TRIGGERS = {
    'GREETING': 'trigger_greeting',
    'ERROR_MINOR': 'trigger_error_minor',
    'ERROR_MAJOR': 'trigger_error_major',
    'LOW_BATTERY': 'trigger_low_battery',
    'START_MOVE': 'trigger_start_move',
    'STOP_MOVE': 'trigger_stop_move',
    'REVERSE': 'trigger_reverse',
    'SPEAKING': 'trigger_speaking',
}


def input_thread(idle_state):
    """Thread für interaktive Benutzereingaben."""
    print("\n" + "="*55)
    print("         INTERAKTIVER STATE TRIGGER")
    print("="*55)
    print("\nVerfügbare States (mit Sound):")
    print("-" * 55)
    print("  GREETING     - Begrüßung")
    print("  ERROR_MINOR  - Leichter Fehler")
    print("  ERROR_MAJOR  - Schwerer Fehler")
    print("  LOW_BATTERY  - Niedriger Akkustand")
    print("  START_MOVE   - Bewegung startet")
    print("  STOP_MOVE    - Bewegung stoppt")
    print("  REVERSE      - Rückwärtsfahrt")
    print("  SPEAKING     - Sprachausgabe")
    print("-" * 55)
    print("  QUIT         - Beenden")
    print("="*55 + "\n")
    
    while not rospy.is_shutdown():
        try:
            user_input = input("State > ").strip().upper()
            
            if user_input == 'QUIT' or user_input == 'Q':
                rospy.loginfo("[INPUT] Shutdown requested")
                rospy.signal_shutdown("User requested shutdown")
                break
            
            if user_input == '':
                continue
            
            # Prüfe ob State existiert
            if user_input in STATE_TRIGGERS:
                if idle_state is not None:
                    trigger = STATE_TRIGGERS[user_input]
                    rospy.loginfo(f"[INPUT] Triggering {user_input} state")
                    idle_state.set_trigger(trigger)
                else:
                    print("[ERROR] IDLE State nicht verfügbar")
            else:
                print(f"[WARN] Unbekannter State: {user_input}")
                print("Verfügbar:", ", ".join(STATE_TRIGGERS.keys()))
                
        except EOFError:
            break
        except Exception as e:
            rospy.logwarn(f"[INPUT] Error: {e}")
            break


def main():
    """Test der State Machine mit interaktivem Trigger."""
    print("="*55)
    print("    TEAM SIGNALE — SMACH STATE MACHINE TEST")
    print("="*55)
    
    # ROS Node initialisieren
    rospy.init_node('test_state_machine', anonymous=False)
    rospy.loginfo("[TEST] Starting State Machine Test")
    
    # State Machine erstellen
    sm = create_state_machine()
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
