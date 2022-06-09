#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Robot:
    def __init__(self):
        """Initialize ROS publisher & ROS subscriber."""
        num_legs = 6
        joints_per_leg = 3

        # Initialize joint publishers
        # /millihex/leg#_joint#_position_controller/command
        for i in range(num_legs):
            for j in range(joints_per_leg):
                leg_number = i+1
                joint_number = j+1
                publisher_name = f"self.leg{leg_number}_joint{joint_number}_pub"
                publisher_topic = f"/millihex/leg{leg_number}" \
                    f"_joint{joint_number}_position_controller/command"
                start_publisher_command = f"{publisher_name} = " \
                    f"rospy.Publisher('{publisher_topic}', Float64, queue_size=1, latch=True)"
                exec(start_publisher_command)

        # Subscribe to millihex/joint_states topic
        self.subscriber = rospy.Subscriber("/millihex/joint_states", \
            JointState, self.callback)

        # Keep track of all robot joint positions
        self.joint_positions = Float64()


    def callback(self, ros_data):
        """Subscriber callback function of /millihex/joint_states topic.
        Here, the joint state positions are updated every 2 seconds."""
        self.joint_positions = ros_data.position        # stored as a tuple
        rate = rospy.Rate(0.5)                          # update every 2 sec
        rate.sleep()


    def get_joint_positions(self):
        """Returns current joint positions of robot"""
        return self.joint_positions


    def publish_joint_position(self, leg_number, joint_number, joint_position):
        """Publishes new joint_position to specified leg_number and joint_number."""
        publisher_name = f"leg{leg_number}_joint{joint_number}_pub"
        publish_command = f"self.{publisher_name}.publish({joint_position})"
        print(f"publish_command = {publish_command}")
        exec(publish_command)