#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class joint:
    def __init__(self):
        """Initialize ROS publisher & ROS subscriber"""
        # Publish to joint position controller command topic
        self.joint_pub = rospy.Publisher("/millihex/leg1_joint1_position_controller/command", Float64, queue_size = 1)

        # Subscribe to robot joint_states topic
        self.subscriber = rospy.Subscriber("/camera/image/compressed", JointState, self.callback)

        # Store joint position
        self.position = Float64()

    def callback(self, ros_data):
        """Subscriber callback function of joint_states topic
        Here joint state positions are updated"""
        print("Joint Position: %s", ros_data.position)

    def get_joint_position(self):
        """Returns current joint position angle"""
        return self.position


def main():
    """Initializes ROS node for a single leg joint"""
    try:
        joint()
        rospy.init_node('leg_joint', anonymous=True)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())