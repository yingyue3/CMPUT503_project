#!/usr/bin/env python3

# potentially useful for question - 1.1 - 1.4 and 2.1

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import numpy as np

import cv2 as cv
from cv_bridge import CvBridge

class LaneDetectionNode(DTROS):
    def __init__(self, node_name):
        super(LaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # add your code here
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_info = f"/{self._vehicle_name}/camera_node/camera_info"
        # camera calibration parameters (intrinsic matrix and distortion coefficients)
        self.K = None
        self.D = None
        self.sub_info = rospy.Subscriber(self._camera_info, CameraInfo, self.callback_info)
        
        # color detection parameters in HSV format
        # Set range for red color
        self.red_lower = np.array([135, 80, 100], np.uint8) 
        self.red_upper = np.array([190, 255, 255], np.uint8) 


        # Set range for green color 
        self.green_lower = np.array([36, 52, 72], np.uint8) 
        self.green_upper = np.array([102, 180, 180], np.uint8) 

        # Set range for blue color 
        self.blue_lower = np.array([110, 80, 80], np.uint8) 
        self.blue_upper = np.array([120, 200, 200], np.uint8) 

        # while bound
        self.white_lower = np.array([0, 0, 180], np.uint8) 
        self.white_upper = np.array([180, 40, 255], np.uint8)

        # yellow bound
        self.yellow_lower = np.array([20, 80, 100], np.uint8) 
        self.yellow_upper = np.array([40, 255, 255], np.uint8) 

        # Set range for mat
        self.mat_lower = np.array([20, 40, 40], dtype=np.uint8)   # Lower bound of mat color
        self.mat_upper = np.array([40, 255, 255], dtype=np.uint8)
        

        # yellow bound
        # self.yellow_lower = np.array([25, 50, 70], np.uint8) 
        # self.yellow_upper = np.array([35, 255, 255], np.uint8) 

        # white bound
        # self.white_lower = np.array([0, 0, 177], np.uint8)
        # self.white_upper = np.array([180, 40, 190], np.uint8)
        
        # initialize bridge and subscribe to camera feed
        self._bridge = CvBridge()
        self.disorted_image = None
        self.color_detect_image = None
        self.black_detect_image = None
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback_image)

        # lane detection publishers
        self._custom_topic = f"/{self._vehicle_name}/custom_node/image/compressed"
        self.pub = rospy.Publisher(self._custom_topic, Image) # queue_size=10

        self._custom_topic_lane = f"/{self._vehicle_name}/custom_node/image/black"
        self.pub_lane = rospy.Publisher(self._custom_topic_lane, Image) # queue_size=10

        # LED
        
        # ROI vertices
        
        # define other variables as needed
        self.image_publish()

    def callback_info(self, msg):
        rate = rospy.Rate(1)
        # https://stackoverflow.com/questions/55781120/subscribe-ros-image-and-camerainfo-sensor-msgs-format
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        # https://github.com/IntelRealSense/realsense-ros/issues/709ss
        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)
        # rospy.loginfo("Camera parameters received.")
        rate.sleep()

    def callback_image(self, msg):
        # add your code here
        
        # convert compressed image to CV2
        rate = rospy.Rate(3)
        if self.K is None:
            return
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # undistort image
        dst = self.undistort_image(image)
        # preprocess image
        imageFrame = self.preprocess_image(dst).astype(np.uint8)
        self.disorted_image = imageFrame
        # rospy.loginfo("Image Calibrated")
        # detect lanes - 2.1 

        # publish lane detection results
        
        # detect lanes and colors - 1.3
        # publish undistorted image
        self.color_detect_image = self.detect_lane_color(self.disorted_image)
        self.black_detect_image = self.detect_lane(self.disorted_image)

        # control LEDs based on detected colors

        # anything else you want to add here
        rate.sleep()

        

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
        new_width = 400
        new_height = 300
        resized_image = cv.resize(raw_image, (new_width, new_height), interpolation = cv.INTER_AREA)
        blurred_image = cv.blur(resized_image, (5, 5)) 
        return blurred_image
    
    def detect_lane_color(self, imageFrame):
        # rate = rospy.Rate(3)
        if self.K is None:
            return None
        hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)

        # Set range for red color and 
        # define mask 
        red_mask = cv.inRange(hsvFrame, self.red_lower, self.red_upper) 

        # Set range for green color and 
        # define mask 
        green_mask = cv.inRange(hsvFrame, self.green_lower, self.green_upper) 

        # Set range for blue color and 
        # define mask 
        blue_mask = cv.inRange(hsvFrame, self.blue_lower, self.blue_upper) 

        # Morphological Transform, Dilation 
        # for each color and bitwise_and operator 
        # between imageFrame and mask determines 
        # to detect only that particular color 
        kernel = np.ones((5, 5), "uint8") 
        
        # For red color 
        red_mask = cv.dilate(red_mask, kernel) 
        res_red = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = red_mask) 
        
        # For green color 
        green_mask = cv.dilate(green_mask, kernel) 
        res_green = cv.bitwise_and(imageFrame, imageFrame, 
                                    mask = green_mask) 
        
        # For blue color 
        blue_mask = cv.dilate(blue_mask, kernel) 
        res_blue = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = blue_mask) 

        # Creating contour to track red color 
        contours, hierarchy = cv.findContours(red_mask, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            area = cv.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv.boundingRect(contour) 
                imageFrame = cv.rectangle(imageFrame, (x, y), 
                                        (x + w, y + h), 
                                        (0, 0, 255), 2) 
                
                cv.putText(imageFrame, "Red Colour", (x, y), 
                            cv.FONT_HERSHEY_SIMPLEX, 1.0, 
                            (0, 0, 255)) 
        # Creating contour to track green color 
        contours, hierarchy = cv.findContours(green_mask, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            area = cv.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv.boundingRect(contour) 
                imageFrame = cv.rectangle(imageFrame, (x, y), 
                                        (x + w, y + h), 
                                        (0, 255, 0), 2) 
                
                cv.putText(imageFrame, "Green Colour", (x, y), 
                            cv.FONT_HERSHEY_SIMPLEX, 
                            1.0, (0, 255, 0)) 

        # Creating contour to track blue color 
        contours, hierarchy = cv.findContours(blue_mask, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE) 
        for pic, contour in enumerate(contours): 
            area = cv.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv.boundingRect(contour) 
                imageFrame = cv.rectangle(imageFrame, (x, y), 
                                        (x + w, y + h), 
                                        (255, 0, 0), 2) 
                
                cv.putText(imageFrame, "Blue Colour", (x, y), 
                            cv.FONT_HERSHEY_SIMPLEX, 
                            1.0, (255, 0, 0)) 
        return imageFrame  
    
    def detect_lane(self,imageFrame):
        # add your code here
        # potentially useful in question 2.1

        height = imageFrame.shape[0]
        imageFrame = imageFrame[height//3:-height//5, :, :]


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

        # yellow_mask = cv.bitwise_and(yellow_mask, mat_mask)
        # filtered_white_mask = np.zeros_like(white_mask)

        # for row in range(white_mask.shape[0]):  
        #     yellow_pixels = np.where(yellow_mask[row, :] > 0)[0]  

        #     if len(yellow_pixels) > 3:  
        #         yellow_pos = yellow_pixels[-1]  

        #         yellow_mask[row, :yellow_pixels[-2]+1] = 0
                # yellow_mask[row, yellow_pixels[-1]+1:] = 0
                
                # white_pixels = np.where(white_mask[row, yellow_pos:] > 0)[0]  

                # if len(white_pixels) > 0:
                    
                #     selected_white_pixels = white_pixels[:1] 
                #     filtered_white_mask[row, selected_white_pixels] = 100 

        
        # lane_mask = cv.bitwise_or(yellow_mask, white_mask)
        lane_mask = np.zeros_like(white_mask)  

        # Set yellow pixels to gray (128)
        lane_mask[yellow_mask > 0] = 128  

        # Set white pixels to white (255)
        # lane_mask[white_mask > 0] = 255 

        # Apply the mask to the frame
        # lane_detection = cv.bitwise_and(imageFrame, imageFrame, mask=lane_mask)

        # contours, hierarchy = cv.findContours(lane_mask, 
        #                                     cv.RETR_TREE, 
        #                                     cv.CHAIN_APPROX_SIMPLE) 
        
        # for pic, contour in enumerate(contours): 
        #     area = cv.contourArea(contour) 
        #     if(area > 300): 
        #         x, y, w, h = cv.boundingRect(contour) 
        #         self.color_detect_image = cv.rectangle(imageFrame, (x, y), 
        #                                 (x + w, y + h), 
        #                                 (0, 0, 255), 2) 
                
        #         cv.putText(self.color_detect_image, "Colour", (x, y), 
        #                     cv.FONT_HERSHEY_SIMPLEX, 1.0, 
        #                     (0, 0, 255)) 


        return lane_mask
    

    # add other functions as needed
    def image_publish(self):
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():       
            if self.black_detect_image is not None:
                # rospy.loginfo('publishing image')
                # image_msg = self._bridge.cv2_to_imgmsg(self.color_detect_image, encoding="bgr8")
                black_msg = self._bridge.cv2_to_imgmsg(self.black_detect_image, encoding="8UC1")
                # self.pub.publish(image_msg)
                self.pub_lane.publish(black_msg)
            #self.pub.publish(self.raw_image)
        rate.sleep()

if __name__ == '__main__':
    node = LaneDetectionNode(node_name='lane_detection_node')
    rospy.spin()