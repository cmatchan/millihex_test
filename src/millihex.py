#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Robot:
    def __init__(self, num_legs, joints_per_leg):
        """Initialize ROS publishers & ROS subscribers."""
        self.num_legs = num_legs
        self.joints_per_leg = joints_per_leg
        self.num_joint_publishers = 0       # Number of joint publishers started

        def start_joint_position_controller_publishers(self):
            """Initialize joint publishers for the topic
            /millihex/leg#_joint#_position_controller/command."""
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

                    # Execute command string as a function
                    exec(start_publisher_command)

                    check_num_pub_connections = f"self.{publisher_name}.get_num_connections() < 1"
                    while exec(check_num_pub_connections):
                        rate = rospy.Rate(1)
                        rate.sleep()

                    self.num_joint_publishers = self.num_joint_publishers + 1

        # Initialize all joint position controllers for command topic
        start_joint_position_controller_publishers(self)

        # Subscribe to millihex/joint_states topic
        self.subscriber = rospy.Subscriber("/millihex/joint_states", \
            JointState, self.callback)

        # Stores robot joint positions
        self.joint_positions = Float64()


    def callback(self, ros_data):
        """Subscriber callback function of /millihex/joint_states topic."""
        self.joint_positions = ros_data.position        # Joint positions stored as tuple
        # print(f"Subscriber callback = {self.joint_positions}\n")
        rate = rospy.Rate(1)
        rate.sleep()


    def get_joint_position(self, leg_number, joint_number):
        """Returns the current position of joint_number in leg_number."""
        joint_index = (leg_number - 1) * self.joints_per_leg + (joint_number - 1)
        return self.joint_positions[joint_index]


    def get_all_joint_positions(self):
        """Returns a tuple of all robot joint positions."""
        return self.joint_positions


    def set_joint_position(self, leg_number, joint_number, joint_position):
        """Publishes joint_position to specified leg_number, joint_number."""
        # Name of joint publisher = "leg#_joint#_pub"
        publisher_name = f"leg{leg_number}_joint{joint_number}_pub"

        # Publish joint_position to joint publisher
        publish_command = f"self.{publisher_name}.publish({joint_position})"

        exec(publish_command)       # Execute string as a function


    def leg_stance(self, leg_number, high_stance = 0):
        """Commands leg to low or high stance position.
        Low stance: high_stance = 0
        High stance: high_stance = 1"""
        joint_angle = 0.4       # Default to low stance joint angle

        # Change to higher joint angle for high stance
        if high_stance == 1:
            joint_angle = 0.8
        else:
            raise Exception("Input error: Use 0 for low stance, 1 for high stance")

        # LHS leg joints have flipped axes
        if leg_number > (int) (self.num_legs / 2):
            joint_angle = (-1) * joint_angle

        print(f"joint_angle = {joint_angle}")

        # Use joints 1 and 3 for vertical positioning
        self.set_joint_position(leg_number, 1, joint_angle)
        self.set_joint_position(leg_number, 3, joint_angle)


    def tripod_stance(self):
        #TODO
        pass

    def leg_swing(self):
        #TODO
        pass


    def stand_up(self):
        """Commands robot to stand up with all legs in low stance position."""
        for i in range(self.num_legs):
            leg_number = i+1        # Robot leg indexes from 1
            self.leg_stance(leg_number, high_stance = 1)
    

    def lay_down(self):
        #TODO
        pass


    def robot_leg_foward(self, leg_number):
        # TODO
        pass