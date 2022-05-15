#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('joint_angle_listener', anonymous=True)
    rospy.Subscriber('/millihex/joint_states')

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass