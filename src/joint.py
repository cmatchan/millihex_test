#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class joint:
    def __init__(self):
        """Initialize ROS publisher & ROS subscriber"""

        # Publish to joint position controller command topic
        self.joint_pub = rospy.Publisher("/millihex/leg1_joint1_position_controller/command", Float64)

        # Subscribe to robot joint_states topic
        self.subscriber = rospy.Subscriber("/camera/image/compressed", JointState, self.callback,  queue_size = 1)

    def callback(self, ros_data):
        """Subscriber callback function of joint_states topic
        Here images get converted and features detected"""

        if VERBOSE :
            print('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        #### Feature detectors using CV2 #### 
        # "","Grid","Pyramid" + 
        # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        method = "GridFAST"
        feat_det = cv2.FeatureDetector_create(method)
        time1 = time.time()

        # convert np image to grayscale
        featPoints = feat_det.detect(
            cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
        time2 = time.time()
        if VERBOSE :
            print '%s detector found: %s points in: %s sec.'%(method,
                len(featPoints),time2-time1)

        for featpoint in featPoints:
            x,y = featpoint.pt
            cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
        
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)


def main(args):
    """Initializes ROS node for a single leg joint"""
    
    try:
        joint()
        rospy.init_node('leg_joint', anonymous=True)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)