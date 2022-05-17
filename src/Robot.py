#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Robot:
    def __init__(self):
        """Initialize ROS publisher & ROS subscriber"""
        # Publish to joint position controller command topic
        self.joint_pub = rospy.Publisher("/millihex/leg1_joint1_position_controller/command", Float64, queue_size = 1)

        # Subscribe to robot joint_states topic
        self.subscriber = rospy.Subscriber("/millihex/joint_states", JointState, self.callback)

        # Store joint position
        self.position = Float64()

    def callback(self, ros_data):
        """Subscriber callback function of joint_states topic
        Here joint state positions are updated"""
        self.position = ros_data.position
        rate = rospy.Rate(0.5)
        rate.sleep()

    def get_joint_positions(self):
        """Returns current joint positions for robot"""
        return self.position

    def compute_fk():
        """Returns forward kinematics for legs"""

    def compute_ik():
        """Returns inverse kinematics for legs"""


def main():
    """Initializes ROS node for robot"""
    try:
        rospy.init_node('robot_definition', anonymous=True)
        robot = Robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())