#!/usr/bin/env python3
"""
Test-Script für die SMACH State Machine.
Kann mit rosrun signal_project test_state_machine.py gestartet werden.
"""

import rospy
import smach_ros

from signal_project.state_machine.state_machine import create_state_machine, get_idle_state
from signal_project.state_machine.signal_state_defs import SignalState


def main():
    """Test der State Machine mit manuellem Trigger."""
    print("=== TEAM SIGNALE — SMACH STATE MACHINE TEST ===")
    
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
    
    print("\nVerfügbare Trigger:")
    print("  - greeting (trigger_greeting)")
    print("  - quit (beenden)")
    
    # Timer für Test: Nach 5 Sekunden GREETING triggern
    def trigger_greeting_callback(event):
        if idle_state is not None:
            rospy.loginfo("[TEST] Auto-triggering GREETING state")
            idle_state.set_trigger('trigger_greeting')
    
    rospy.Timer(rospy.Duration(5.0), trigger_greeting_callback, oneshot=True)
    
    try:
        # State Machine ausführen
        rospy.loginfo("[TEST] Executing State Machine...")
        outcome = sm.execute()
        rospy.loginfo(f"[TEST] State Machine finished with outcome: {outcome}")
    except rospy.ROSInterruptException:
        rospy.loginfo("[TEST] ROS Interrupt received")
    finally:
        sis.stop()
        rospy.loginfo("[TEST] Test completed")


if __name__ == "__main__":
    main()
