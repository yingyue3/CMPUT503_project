#!/usr/bin/env python3

# potentially useful for part 1 of exercise 4

# import required libraries
import rospy
import os
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import numpy as np
from duckietown_msgs.msg import LEDPattern 

import cv2 as cv
from cv_bridge import CvBridge
import dt_apriltags as aptag
from std_msgs.msg import Header, ColorRGBA, Int32, String

class ApriltagNode(DTROS):

    def __init__(self, node_name):
        super(ApriltagNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # add your code here
        # while bound
        self.white_lower = np.array([0, 0, 180], np.uint8) 
        self.white_upper = np.array([180, 40, 255], np.uint8)

        # yellow bound
        self.yellow_lower = np.array([20, 80, 100], np.uint8) 
        self.yellow_upper = np.array([40, 255, 255], np.uint8) 

        # Set range for mat
        self.mat_lower = np.array([20, 40, 40], dtype=np.uint8)   # Lower bound of mat color
        self.mat_upper = np.array([40, 255, 255], dtype=np.uint8)

        # color detection parameters in HSV format
        # Set range for red color
        self.red_lower = np.array([110, 80, 80], np.uint8) 
        self.red_upper = np.array([120, 200, 200], np.uint8) 

        # call navigation control node
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_info = f"/{self._vehicle_name}/camera_node/camera_info"

        # initialize dt_apriltag detector
        # https://github.com/duckietown/lib-dt-apriltags

        # subscribe to camera feed
        self.K = None
        self.D = None
        self.sub_info = rospy.Subscriber(self._camera_info, CameraInfo, self.callback_info)
        self._bridge = CvBridge()
        self.disorted_image = None
        self.color_detect_image = None
        self.black_detect_image = None
        self.imageFrame = None
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback_image)

        self.led_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"
        self.led_pub = rospy.Publisher(self.led_topic, LEDPattern, queue_size=1)

        # define other variables as needed
        # lane detection publishers
        self._custom_topic = f"/{self._vehicle_name}/custom_node/image/compressed"
        self.pub = rospy.Publisher(self._custom_topic, Image) # queue_size=10

        self._custom_topic_augmented_image = f"/{self._vehicle_name}/custom_node/image/black"
        self.pub_augmented_image = rospy.Publisher(self._custom_topic_augmented_image , Image) # queue_size=10


        self.aprilid = 1000
        self._custom_topic_apriltag = f"/{self._vehicle_name}/control_node/apriltag"
        self.pub_apriltag = rospy.Publisher(self._custom_topic_apriltag, Int32)

        self.gray = None

        self.red_lane_message = "No"
        self._custom_topic_red_lane = f"/{self._vehicle_name}/control_node/red_lane"
        self.pub_red_lane = rospy.Publisher(self._custom_topic_red_lane, String)
        # self.publish_augmented_img()
        
    
    def callback_info(self, msg):
        rate = rospy.Rate(1)
        # https://stackoverflow.com/questions/55781120/subscribe-ros-image-and-camerainfo-sensor-msgs-format
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        # https://github.com/IntelRealSense/realsense-ros/issues/709ss
        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)
        rate.sleep()

    # def callback_image(self, msg):
    #     # add your code here
        
    #     # convert compressed image to CV2
    #     # rate = rospy.Rate(30)
    #     start = rospy.Time.now()
    #     if self.K is None:
    #         return
    #     image = self._bridge.compressed_imgmsg_to_cv2(msg)
    #     # undistort image
    #     dst = self.undistort_image(image)
    #     # preprocess image
    #     self.disorted_image = dst
    #     imageFrame = self.preprocess_image(dst).astype(np.uint8)
    #     # detect lanes - 2.1 

    #     # publish lane detection results
        
    #     # detect lanes and colors - 1.3
    #     # publish undistorted image
    #     # self.color_detect_image = self.detect_red_lane(cv.blur(imageFrame, (5, 5)))
    #     self.color_detect_image = self.detect_red_lane(imageFrame)
    #     # self.black_detect_image = self.detect_lane(self.disorted_image)
    #     black_msg = self._bridge.cv2_to_imgmsg(self.color_detect_image, encoding="bgr8")
    #     self.pub.publish(black_msg)
    #     self.process_image()
    #     self.detect_tag()
    #     end = rospy.Time.now()
    #     rospy.loginfo("Detection time: %f sec", (end - start).to_sec())

    def callback_image(self, msg):
        # add your code here
        
        # convert compressed image to CV2
        # rate = rospy.Rate(30)
        # start = rospy.Time.now()
        if self.K is None:
            return
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # undistort image
        dst = self.undistort_image(image)
        # preprocess image
        self.disorted_image = dst
        self.imageFrame = self.preprocess_image(dst).astype(np.uint8)
        # detect lanes - 2.1 

        # end = rospy.Time.now()
        # rospy.loginfo("Detection time: %f sec", (end - start).to_sec())
    
    def start(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.imageFrame is None:
                continue
            # start = rospy.Time.now()

            # Red line detection code
            self.color_detect_image = self.detect_red_lane(self.imageFrame)
            # self.black_detect_image = self.detect_lane(self.disorted_image)
            black_msg = self._bridge.cv2_to_imgmsg(self.color_detect_image, encoding="bgr8")
            self.pub.publish(black_msg)
            # rospy.loginfo(self.red_lane_message)
            self.pub_red_lane.publish(self.red_lane_message)

            # PID control stuff
            self.black_detect_image = self.detect_lane(cv.blur(self.disorted_image, (5, 5)))
            black_msg = self._bridge.cv2_to_imgmsg(self.black_detect_image, encoding="8UC1")
            self.pub_augmented_image.publish(black_msg)
            # image_msg = self._bridge.cv2_to_imgmsg(self.gray, encoding="8UC1")
            # self.pub_augmented_image.publish(image_msg)
            # end = rospy.Time.now()
            # rospy.loginfo("Publish time: %f sec", (end - start).to_sec())

    def undistort_image(self, image):
        # convert JPEG bytes to CV image
        # rate = rospy.Rate(3)
        if self.K is None:
            return
        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        h,w = image.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
        dst = cv.undistort(image, self.K, self.D, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        # rospy.loginfo("Image Calibrated")
        return dst
        # rate.sleep()
    
    def preprocess_image(self, raw_image):
        # height = raw_image.shape[0]
        # weight = raw_image.shape[1]
        # rospy.loginfo(height//3)
        # rospy.loginfo(weight//3)
        resized_image = raw_image[200:339+100, 0:450]
        # new_width = 400
        # new_height = 300
        # resized_image = cv.resize(raw_image, (new_width, new_height), interpolation = cv.INTER_AREA)
        blurred_image = cv.blur(resized_image, (5, 5)) 
        return resized_image
    
    def detect_red_lane(self, imageFrame):
        hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)
        # red mask
        # rospy.loginfo("line detecting")
        mask = cv.inRange(hsvFrame, self.red_lower, self.red_upper) 

        kernel = np.ones((5, 5), "uint8") 
        mask = cv.dilate(mask, kernel) 
        res = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = mask) 
        contours, hierarchy = cv.findContours(mask, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE)
        red_lane = False
        for pic, contour in enumerate(contours): 
            area = cv.contourArea(contour) 
            if(area > 100): 
                x, y, w, h = cv.boundingRect(contour) 
                # rospy.loginfo(y+h)
                # imageFrame = cv.rectangle(imageFrame, (x, y), 
                #                         (x + w, y + h), 
                #                         (0, 0, 255), 2) 
                
                # cv.putText(imageFrame, "Colour", (x, y), 
                #             cv.FONT_HERSHEY_SIMPLEX, 1.0, 
                #             (0, 0, 255))
                red_lane = True
        if red_lane:
            self.red_lane_message = "Yes"
        else:
            self.red_lane_message = "No"
        return imageFrame

    
    def detect_lane(self, imageFrame):
        # add your code here
        # potentially useful in question 2.1

        height = imageFrame.shape[0]
        imageFrame = imageFrame[height//2:-height//5, :, :]


        imageFrame = cv.GaussianBlur(imageFrame, (5, 5), 0)

        kernel = np.ones((5, 5), "uint8") 

        hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)

        white_mask = cv.inRange(hsvFrame, self.white_lower, self.white_upper) 
        yellow_mask = cv.inRange(hsvFrame, self.yellow_lower, self.yellow_upper) 
        mat_mask = cv.inRange(hsvFrame, self.mat_lower, self.mat_upper)

        # For white color 
        white_mask = cv.dilate(white_mask, kernel) 
        res_white = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = white_mask) 
        
        # For yellow color 
        yellow_mask = cv.dilate(yellow_mask, kernel) 
        res_yellow = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = yellow_mask) 


        lane_mask = np.zeros_like(white_mask)  

        # Set yellow pixels to gray (128)
        # lane_mask[yellow_mask > 0] = 128  

        # Set white pixels to white (255)
        lane_mask[white_mask > 0] = 255 

        return lane_mask

    def publish_augmented_img(self):
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():       
            if self.gray is not None:
                image_msg = self._bridge.cv2_to_imgmsg(self.gray, encoding="8UC1")
                self.pub_augmented_image.publish(image_msg)
                
           
        rate.sleep()
        pass

    def publish_leds(self, x):      
        if self.gray is not None:
            msg = LEDPattern()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            color_msg = ColorRGBA()
            color_msg.r, color_msg.g, color_msg.b, color_msg.a = x


            # Set LED colors
            msg.rgb_vals = [color_msg] * 5
            self.led_pub.publish(msg) 
        pass


if __name__ == '__main__':
    # create the node
    node = ApriltagNode(node_name='apriltag_detector_node')
    node.start()

    rospy.spin()
    
