#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Robot:
    def __init__(self, num_legs, joints_per_leg):
        """Initialize ROS publishers & ROS subscribers"""
        # Initialize all joint position controllers for command topic
        start_all_joint_position_controller_publishers()

        # Subscribe to millihex/joint_states topic
        self.subscriber = rospy.Subscriber("/millihex/joint_states", \
            JointState, self.callback)

        # Stores robot joint positions
        self.joint_positions = Float64()
    
        def start_all_joint_position_controller_publishers(self):
            """ Initialize joint publishers for the topic
            /millihex/leg#_joint#_position_controller/command """
            for i in range(self.num_legs):
                for j in range(self.joints_per_leg):
                    leg_number = i+1        # Robot leg indexes from 1
                    joint_number = j+1      # Robot joint indexes from 1

                    # Name of joint publisher = "leg#_joint#_pub"
                    publisher_name = f"leg{leg_number}_joint{joint_number}_pub"

                    # Name of topic = "/millihex/leg#_joint#_position_controller/command"
                    publisher_topic = f"/millihex/leg{leg_number}" \
                        f"_joint{joint_number}_position_controller/command"

                    # Initialize publisher command
                    start_publisher_command = f"self.{publisher_name} = " \
                        f"rospy.Publisher('{publisher_topic}', Float64, queue_size=1, latch=True)"

                    exec(start_publisher_command)       # Execute string as a function


    def callback(self, ros_data):
        """Subscriber callback function of /millihex/joint_states topic.
        Here, the joint state positions are updated every 2 seconds."""
        self.joint_positions = ros_data.position        # stored as a tuple
        rate = rospy.Rate(0.5)                          # update every 2 sec
        rate.sleep()


    def get_single_joint_position(self, leg_number, joint_number):
        # TODO
        pass


    def get_all_joint_positions(self):
        """Returns current joint positions of robot"""
        return self.joint_positions


    def publish_single_joint_position(self, leg_number, joint_number, joint_position):
        """Publishes new joint_position to specified leg_number and joint_number."""
        # Name of joint publisher = "leg#_joint#_pub"
        publisher_name = f"leg{leg_number}_joint{joint_number}_pub"

        # Publish joint_position to joint publisher
        publish_command = f"self.{publisher_name}.publish({joint_position})"

        print(f"publish_command = {publish_command}")
        exec(publish_command)       # Execute string as a function


    def publish_all_joints_position(self, joint_position):
        # TODO
        pass
    

    def robot_stance(self):
        # TODO
        pass


    def robot_leg_foward(self, leg_number):
        # TODO
        pass