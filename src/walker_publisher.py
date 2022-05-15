#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def talker():
    rospy.init_node('joint_topic_publisher', anonymous=True)
    pub = rospy.Publisher('/millihex/leg1_joint1_position_controller/command', Float64, queue_size=1)

    rate = rospy.Rate(2)                        # Set a publish rate of 2 Hz
    phi = 0.0                                   # Initialize joint angle position

    rospy.loginfo("Started joint_topic_publisher\n")

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass