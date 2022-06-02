#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Robot:
    def __init__(self):
        """Initialize ROS publisher & ROS subscriber."""

        # Publish to /millihex/joint1_position1_controller/command topic
        self.leg1_joint1_pub = rospy.Publisher( \
            "/millihex/leg1_joint1_position_controller/command", \
            Float64, queue_size = 1)

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

    def publish_joint_positions(self, leg_number, joint_number, joint_position):
        """Publishes new joint_position to specified leg_number and joint_number."""
        
        publisher_name = f"leg{leg_number}_joint{joint_number}_pub"
        publish_command = f"self.{publisher_name}.publish({joint_position})"
        print(f"publish_command = {publish_command}")
        exec(publish_command)