#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType

from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

class MotionNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MotionNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_info = f"/{self._vehicle_name}/camera_node/camera_info"

        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.twisted_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        
        self._v = 0
        self._omega = 0
        
        
        self._bridge = CvBridge()

        self.sub_info = rospy.Subscriber(self._camera_info, CameraInfo, self.callback_info)
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback_image)

        self.K = None
        self.D = None

        self.block_length = 10
        self.undisorted_image = None
        self.gray = None

        self.control_type = "PID"
        self.proportional_gain = 0.05
        self.derivative_gain = 0.03
        self.integral_gain = 0.001

        #color detection
        self.white_lower = np.array([0, 0, 180], np.uint8) 
        self.white_upper = np.array([180, 40, 255], np.uint8)

        self.error = 0
        self.prev_error = 0


    def callback_info(self, msg):
        # https://stackoverflow.com/questions/55781120/subscribe-ros-image-and-camerainfo-sensor-msgs-format
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        # https://github.com/IntelRealSense/realsense-ros/issues/709ss
        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)

    def callback_image(self, msg):
        if self.K is None:
            return
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        h,w = image.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
        dst = cv.undistort(image, self.K, self.D, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        self.undisorted_image = self.image_preprocess(dst)
        self.gray = self.calc_error(self.undisorted_image)

    def image_preprocess(self, img):
        new_width = 400
        new_height = 300
        resized_image = cv.resize(img (new_width, new_height), interpolation = cv.INTER_AREA)
        blurred_image = cv.blur(resized_image, (5, 5)) 
        return blurred_image
    
    def calc_error(self, imageFrame):
        height = imageFrame.shape[0]
        imageFrame = imageFrame[height//3:-height//5, :, :]


        imageFrame = cv.GaussianBlur(imageFrame, (5, 5), 0)

        kernel = np.ones((5, 5), "uint8") 

        hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)

        white_mask = cv.inRange(hsvFrame, self.white_lower, self.white_upper) 

        # For white color 
        white_mask = cv.dilate(white_mask, kernel) 
        res_white = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = white_mask) 
        
        # For yellow color 
        yellow_mask = cv.dilate(yellow_mask, kernel) 
        res_yellow = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = yellow_mask) 

        lane_mask = np.zeros_like(white_mask)  
        lane_mask[white_mask > 0] = 255 


        contours, hierarchy = cv.findContours(lane_mask, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE) 
        
        if len(contours)>0:
            # Sort contours by area in descending order and pick the top two
            max_contour = sorted(contours, key=cv.contourArea, reverse=True)[0]

            x_values = max_contour[:, 0, 0]  # Extracting x-coordinates

            # Compute the average x-coordinate
            avg_x = np.mean(x_values)


        self.error = avg_x- lane_mask.shape[1]/2.0 + self.calibration

        return lane_mask
        
    def publish_twisted(self, v, omega):
        message = Twist2DStamped(v=v, omega=omega)
        self.twisted_publisher.publish(message)

    def pid_controller(self):
        self.history = np.roll(self.history, shift=-1, axis=1)  # Shift all values left
        self.history[0, -1] = self.error
        self.integral = np.sum(self.history)
        derivative = self.error - self.prev_error 
        return self.proportional_gain * self.error + self.derivative_gain * derivative + self.integral_gain * self.integral

    def one_block_pid(self):
        distance_traveled = 0

        while distance_traveled < self.block_length:
            control = self.pid_controller()
            self.publish_twisted(v=self._v, omega = -1*control)

            #update distance_traveled

    def straight_one_block(self):
        distance_traveled = 0

        while distance_traveled < self.block_length:
            self.publish_twisted(v=self._v, omega = 0)

            #update distance_traveled

    def arch_counter_clockwise(self):
        distance_traveled = 0

        while distance_traveled < self.block_length:
            self.publish_twisted(v=self._v, omega = np.pi//4)

            #update distance_traveled  

    def turn_inplace_clockwise(self):
        self.publish_twisted(v = 0, omega = np.pi/2)
        self.publish_twisted(v = 0, omega = 0)

    def turn_inplace_counter_clockwise(self):
        self.publish_twisted(v = 0, omega = -np.pi/2)
        self.publish_twisted(v = 0, omega = 0)

    def stop(self):
        self.publish_twisted(v = 0, omega = 0)

    def on_shutdown(self):
        self.publish_twisted(v = 0, omega = 0)

    def run(self):
        #get the sequence of actions from LLM
        pass

if __name__ == '__main__':
    # create the node
    node = MotionNode(node_name='my_publisher_node')
    rospy.Rate(10)
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()