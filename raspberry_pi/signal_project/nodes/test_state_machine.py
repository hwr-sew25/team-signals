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


def input_thread(idle_state):
    """Thread für interaktive Benutzereingaben."""
    print("\n" + "="*50)
    print("INTERAKTIVER STATE TRIGGER")
    print("="*50)
    print("\nVerfügbare Befehle:")
    print("  GREETING  - GREETING State triggern")
    print("  ERROR     - ERROR State triggern (Platzhalter)")
    print("  BATTERY   - LOW_BATTERY State triggern (Platzhalter)")
    print("  BUSY      - BUSY State triggern (Platzhalter)")
    print("  QUIT      - Beenden")
    print("="*50 + "\n")
    
    while not rospy.is_shutdown():
        try:
            user_input = input("State > ").strip().upper()
            
            if user_input == 'QUIT' or user_input == 'Q':
                rospy.loginfo("[INPUT] Shutdown requested")
                rospy.signal_shutdown("User requested shutdown")
                break
            elif user_input == 'GREETING':
                if idle_state is not None:
                    rospy.loginfo("[INPUT] Triggering GREETING state")
                    idle_state.set_trigger('trigger_greeting')
                else:
                    print("[ERROR] IDLE State nicht verfügbar")
            elif user_input == 'ERROR':
                if idle_state is not None:
                    rospy.loginfo("[INPUT] Triggering ERROR state")
                    idle_state.set_trigger('trigger_error')
            elif user_input == 'BATTERY':
                if idle_state is not None:
                    rospy.loginfo("[INPUT] Triggering LOW_BATTERY state")
                    idle_state.set_trigger('trigger_low_battery')
            elif user_input == 'BUSY':
                if idle_state is not None:
                    rospy.loginfo("[INPUT] Triggering BUSY state")
                    idle_state.set_trigger('trigger_busy')
            elif user_input == '':
                continue
            else:
                print(f"[WARN] Unbekannter Befehl: {user_input}")
                print("Verfügbar: GREETING, ERROR, BATTERY, BUSY, QUIT")
        except EOFError:
            break
        except Exception as e:
            rospy.logwarn(f"[INPUT] Error: {e}")
            break


def main():
    """Test der State Machine mit interaktivem Trigger."""
    print("="*50)
    print("=== TEAM SIGNALE — SMACH STATE MACHINE TEST ===")
    print("="*50)
    
    # ROS Node initialisieren
    rospy.init_node('test_state_machine', anonymous=False)
    rospy.loginfo("[TEST] Starting State Machine Test")
    
    # State Machine erstellen
    sm = create_state_machine()
    rospy.loginfo("[TEST] State Machine created")
    
    # Introspection Server für Debugging
    sis = smach_ros.IntrospectionServer('test_sm_server', sm, '/TEST_SM')
    sis.start()
    
    # IDLE State für Trigger holen
    idle_state = get_idle_state(sm)
    
    # Input-Thread starten
    input_t = threading.Thread(target=input_thread, args=(idle_state,), daemon=True)
    input_t.start()
    
    try:
        # State Machine ausführen
        rospy.loginfo("[TEST] Executing State Machine... (Warte auf Input)")
        rospy.loginfo("[TEST] Gib 'GREETING' ein um den GREETING State zu testen")
        outcome = sm.execute()
        rospy.loginfo(f"[TEST] State Machine finished with outcome: {outcome}")
    except rospy.ROSInterruptException:
        rospy.loginfo("[TEST] ROS Interrupt received")
    finally:
        sis.stop()
        rospy.loginfo("[TEST] Test completed")


if __name__ == "__main__":
    main()
