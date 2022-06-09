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
        num_legs = 6
        joints_per_leg = 3
        
        for i in range(num_legs):
            for j in range(joints_per_leg):
                leg_number = i+1
                joint_number = j+1
                millihex.publish_joint_position(leg_number, joint_number, 0)

        while not rospy.is_shutdown():
            for i in range(int(num_legs/2)):
                leg_number = i+1
                millihex.publish_joint_position(leg_number, 3, 1)
                millihex.publish_joint_position(int(leg_number+num_legs/2), 3, -1)
            rate.sleep()
            
            
            for i in range(int(num_legs/2)):
                leg_number = i+1
                millihex.publish_joint_position(leg_number, 3, 0)
                millihex.publish_joint_position(int(leg_number+num_legs/2), 3, 0)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())