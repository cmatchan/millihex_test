#!/usr/bin/env python3
import sys
import rospy
from millihex import Robot

def main():
    """Initializes ROS node for robot"""
    try:
        rospy.init_node('robot_definition', anonymous=True)
        millihex = Robot()
        rate = rospy.Rate(0.5)

        while not rospy.is_shutdown():
            millihex.publish_joint_positions(1, 1, 1)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())