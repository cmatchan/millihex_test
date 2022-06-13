#!/usr/bin/env python3
import sys
import rospy
from millihex import Robot

def main():
    """Initializes ROS node for robot"""
    try:
        rospy.init_node('robot_rock', anonymous=True)       # Initialize ROS node
        rate = rospy.Rate(0.5)                              # Set refresh rate

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

        # Reset all joint positions to 0;
        for i in range(num_legs):
            for j in range(joints_per_leg):
                leg_number = i+1                # Robot leg indexes from 1
                joint_number = j+1              # Robot joint indexes from 1

                # Set joint position = 0 to each joint in a leg
                millihex.set_joint_position(leg_number, joint_number, 0)
        
        rate.sleep()
    
        # Test
        millihex.robot_stance()


        # # Testing loop
        # while not rospy.is_shutdown():
        #     for i in range(int(num_legs/2)):
        #         leg_number = i+1
        #         millihex.publish_joint_position(leg_number, 3, 1)
        #         millihex.publish_joint_position(int(leg_number+num_legs/2), 3, -1)
        #     rate.sleep()
            
        #     for i in range(int(num_legs/2)):
        #         leg_number = i+1
        #         millihex.publish_joint_position(leg_number, 3, 0)
        #         millihex.publish_joint_position(int(leg_number+num_legs/2), 3, 0)
        #     rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())