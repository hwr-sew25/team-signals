import rospy
from std_msgs.msg import String
from led_engine import send_led_command

def alarm_state_cb(msg):
    state = msg.data.strip().upper()
    send_led_command(state)

def main():
    rospy.init_node("alarm_listener")
    rospy.Subscriber("/alarm_state", String, alarm_state_cb)
    rospy.loginfo("[ALARM_LISTENER] Ready.")
    rospy.spin()

if __name__ == "__main__":
    main()

