#!/usr/bin/env python3
import os
import sys
sys.path.append(os.path.dirname(__file__))

import rospy
from std_msgs.msg import String
from led_engine import send_led_command

def alarm_state_cb(msg: String):
    state = msg.data.strip().upper()
    if not state:
        return
    send_led_command(state)

def main():
    rospy.init_node("alarm_listener")
    rospy.Subscriber("/alarm_state", String, alarm_state_cb)
    rospy.loginfo("[ALARM_LISTENER] Ready. Listening on /alarm_state")
    rospy.spin()

if __name__ == "__main__":
    main()


