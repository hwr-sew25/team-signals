#!/usr/bin/env python3
"""
ROS Node für die Signal State Machine.

Dieser Node initialisiert ROS und startet die SMACH State Machine.
Enthält keine Business-Logik - diese befindet sich in den States.
"""

import rospy
import smach_ros

from signal_project.state_machine.state_machine import create_state_machine, get_idle_state


def main():
    """Hauptfunktion des ROS Nodes."""
    # ROS Node initialisieren
    rospy.init_node('signal_state_machine', anonymous=False)
    rospy.loginfo("[STATE_MACHINE_NODE] Starting Signal State Machine Node")
    
    # State Machine erstellen
    sm = create_state_machine()
    rospy.loginfo("[STATE_MACHINE_NODE] State Machine created")
    
    # Optional: Introspection Server für Debugging (smach_viewer)
    sis = smach_ros.IntrospectionServer('signal_sm_server', sm, '/SIGNAL_SM')
    sis.start()
    
    # IDLE State für externe Trigger holen
    idle_state = get_idle_state(sm)
    
    # Für minimalen Test: Trigger Greeting nach 3 Sekunden
    def trigger_greeting_callback(event):
        if idle_state is not None:
            rospy.loginfo("[STATE_MACHINE_NODE] Triggering GREETING state")
            idle_state.set_trigger('trigger_greeting')
    
    # Timer für Test-Trigger
    rospy.Timer(rospy.Duration(3.0), trigger_greeting_callback, oneshot=True)
    
    try:
        # State Machine ausführen
        rospy.loginfo("[STATE_MACHINE_NODE] Executing State Machine")
        outcome = sm.execute()
        rospy.loginfo(f"[STATE_MACHINE_NODE] State Machine finished with outcome: {outcome}")
    except rospy.ROSInterruptException:
        rospy.loginfo("[STATE_MACHINE_NODE] ROS Interrupt received")
    finally:
        sis.stop()
        rospy.loginfo("[STATE_MACHINE_NODE] Introspection Server stopped")


if __name__ == '__main__':
    main()

