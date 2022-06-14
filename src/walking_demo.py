#!/usr/bin/env python3
import sys
import rospy
from millihex import Robot

def main():
    """Initializes ROS node for robot"""
    try:
        rospy.init_node('robot_rock', anonymous=True)       # Initialize ROS node
        rate = rospy.Rate(2)        # Set refresh rate

        # Initialize a Robot object to spawn millihex
        num_legs = 6
        joints_per_leg = 3
        millihex = Robot(num_legs, joints_per_leg)

        print("ROBOT INITIALIZED")
        print(f"Joint Position Publisher connections = {millihex.num_joint_publishers}\n")

        print("Waiting for Joint States Subscriber...")
        while millihex.subscriber.get_num_connections() < 1:
            rate = rospy.Rate(1)
            rate.sleep()

        print(f"Joint States Subscriber connections = {millihex.subscriber.get_num_connections()}\n")
        
        millihex.lay_down()
        print(f"stance_state: {millihex.get_stance_state()}")
        rate.sleep()

        millihex.stand_up()
        print(f"stance_state: {millihex.get_stance_state()}")
        rate.sleep()

        # # Test millihex tripod gait
        # while not rospy.is_shutdown():
        #     millihex.tripod_gait()
        #     print(f"stance_state: {millihex.get_stance_state()}")
        #     rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())