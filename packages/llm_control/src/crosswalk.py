#!/usr/bin/env python3

# potentially useful for part 2 of exercise 4

# import required libraries
import rospy
from duckietown.dtros import DTROS, NodeType
import os
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import numpy as np
from duckietown_msgs.msg import LEDPattern 

import cv2 as cv
from cv_bridge import CvBridge
import dt_apriltags as aptag
from std_msgs.msg import Header, ColorRGBA, Int32, String

from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from duckietown_msgs.msg import LEDPattern, WheelEncoderStamped


class CrossWalkNode(DTROS):

    def __init__(self, node_name):
        super(CrossWalkNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

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
        self.blue_lower = np.array([110, 150, 100], np.uint8) 
        self.blue_upper = np.array([120, 250, 200], np.uint8) 
        
        # call navigation control node
        self.control_type = "PID"  # it can be P or PD or PID

        # variables
        self.image_w = 400
        
        # PID gains 
        self.proportional_gain = 0.05
        self.derivative_gain = 0.03
        self.integral_gain = 0.001

        #Straight

        # self.proportional_gain = 0.05
        # self.derivative_gain = 0.03
        # self.integral_gain = 0.001
        
        # control variables
        self.prev_error = 0
        self.history = np.zeros((1,10))
        self.integral = 0
        
        # movement parameters
        self.speed = 0.4
        # self.speed = 0
        self.error = 0
        
        # distance tracking
        #self.calibration = 130

        self.calibration = -181

        self.test = []
        
        # initialize publisher/subscribers
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self.wheel_publisher = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)

        # self.lane_topic = f"/{self._vehicle_name}/custom_node/image/black"

        # self.lane_sub = rospy.Subscriber(self.lane_topic, Image, self.yellow_lane_callback)

        self._bridge = CvBridge()

        # twisted topic
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        self._v = 0
        self._omega = 0
        # construct publisher
        self.twisted_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)


        # subscribe to camera feed
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

        # self.gray = None

        self.blue_lane = False
        self.ducks = False
        # self._custom_topic_red_lane = f"/{self._vehicle_name}/control_node/red_lane"
        # self.pub_red_lane = rospy.Publisher(self._custom_topic_red_lane, String)
        # self.publish_augmented_img()

        # None Lane Following Stuff
        self.red_lane_message = "No"

        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"

        self._ticks_left = 0
        self._ticks_right = 0
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        # self.pub = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)
        
        


        # define other variables as needed
        # robot params
        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)
        self.DISTANCE_PER_TICK = np.pi*2*self._radius/135
        self.start_dist = 0
    
    def callback_left(self, data):
        self._ticks_left = data.data

    def callback_right(self, data):
        self._ticks_right = data.data
    
    
    def callback_info(self, msg):
        rate = rospy.Rate(1)
        # https://stackoverflow.com/questions/55781120/subscribe-ros-image-and-camerainfo-sensor-msgs-format
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        # https://github.com/IntelRealSense/realsense-ros/issues/709ss
        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)
        rate.sleep()
    
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
        resized_image = raw_image[200:339+100, 150:]
        # new_width = 400
        # new_height = 300
        # resized_image = cv.resize(raw_image, (new_width, new_height), interpolation = cv.INTER_AREA)
        blurred_image = cv.blur(resized_image, (5, 5)) 
        return resized_image
    
    def detect_lane(self, imageFrame):
        # add your code here
        # potentially useful in question 2.1

        height = imageFrame.shape[0]
        imageFrame = imageFrame[height//2:-height//5, :, :]


        imageFrame = cv.GaussianBlur(imageFrame, (5, 5), 0)

        kernel = np.ones((5, 5), "uint8") 

        hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)

        white_mask = cv.inRange(hsvFrame, self.white_lower, self.white_upper) 
        # yellow_mask = cv.inRange(hsvFrame, self.yellow_lower, self.yellow_upper) 
        # mat_mask = cv.inRange(hsvFrame, self.mat_lower, self.mat_upper)

        # For white color 
        white_mask = cv.dilate(white_mask, kernel) 
        res_white = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = white_mask) 
        
        # For yellow color 
        # yellow_mask = cv.dilate(yellow_mask, kernel) 
        # res_yellow = cv.bitwise_and(imageFrame, imageFrame, 
        #                         mask = yellow_mask) 


        lane_mask = np.zeros_like(white_mask)  

        # Set yellow pixels to gray (128)
        # lane_mask[yellow_mask > 0] = 128  

        # Set white pixels to white (255)
        lane_mask[white_mask > 0] = 255

        image = lane_mask

        contours, hierarchy = cv.findContours(image, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE) 
        
        
        if len(contours)>0:
            # Sort contours by area in descending order and pick the top two
            max_contour = sorted(contours, key=cv.contourArea, reverse=True)[0]

            x_values = max_contour[:, 0, 0]  # Extracting x-coordinates

            # Compute the average x-coordinate
            avg_x = np.mean(x_values)
         
        # cv.line(image, (xc,yc), (xc+yc, yc), color=255)

        self.error = avg_x- image.shape[1]/2.0 + self.calibration

        # rospy.loginfo(self.error)

        # Contour approach

        return lane_mask
    
    def detect_blue_lane(self, imageFrame):
        hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)
        # red mask
        # rospy.loginfo("line detecting")
        mask = cv.inRange(hsvFrame, self.blue_lower, self.blue_upper) 

        kernel = np.ones((5, 5), "uint8") 
        mask = cv.dilate(mask, kernel) 
        res = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = mask) 
        contours, hierarchy = cv.findContours(mask, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE)
        # self.blue_lane = False
        for pic, contour in enumerate(contours): 
            area = cv.contourArea(contour) 
            if(area > 700): 
                x, y, w, h = cv.boundingRect(contour) 
                # rospy.loginfo(y+h)
                # imageFrame = cv.rectangle(imageFrame, (x, y), 
                #                         (x + w, y + h), 
                #                         (0, 0, 255), 2) 
                
                # cv.putText(imageFrame, "Colour", (x, y), 
                #             cv.FONT_HERSHEY_SIMPLEX, 1.0, 
                #             (0, 0, 255))
                if y + h > 100:
                    self.blue_lane = True
        return imageFrame
    
    def detect_ducks(self, imageFrame):
        hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)
        # red mask
        # rospy.loginfo("line detecting")
        mask = cv.inRange(hsvFrame, self.yellow_lower, self.yellow_upper)

        kernel = np.ones((5, 5), "uint8") 
        mask = cv.dilate(mask, kernel) 
        res = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = mask) 
        contours, hierarchy = cv.findContours(mask, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE)
        self.ducks = False
        for pic, contour in enumerate(contours): 
            area = cv.contourArea(contour) 
            if(area > 700): 
                x, y, w, h = cv.boundingRect(contour) 
                # rospy.loginfo(y+h)
                imageFrame = cv.rectangle(imageFrame, (x, y), 
                                        (x + w, y + h), 
                                        (0, 0, 255), 2) 
                
                cv.putText(imageFrame, "Colour", (x, y), 
                            cv.FONT_HERSHEY_SIMPLEX, 1.0, 
                            (0, 0, 255))
                self.ducks = True
        return imageFrame

    
    def calculate_p_control(self):
        # add your code here
        return self.proportional_gain * self.error

    def calculate_pd_control(self):
        # add your code here
        derivative = self.error - self.prev_error 
        return self.proportional_gain * self.error + self.derivative_gain * derivative
    
    def calculate_pid_control(self):
        # add your code here
        self.history = np.roll(self.history, shift=-1, axis=1)  # Shift all values left
        self.history[0, -1] = self.error
        self.integral = np.sum(self.history)
        derivative = self.error - self.prev_error 
        return self.proportional_gain * self.error + self.derivative_gain * derivative + self.integral_gain * self.integral

    def get_control_output(self):
        if self.control_type == "P":
            control = self.calculate_p_control()
        elif self.control_type == "PD":
            control = self.calculate_pd_control()
        elif self.control_type == "PID":
            control = self.calculate_pid_control()
        else:
            rospy.logwarn("Invalid control type!")
            control = 0.0
        self.prev_error = self.error

        self.publish_cmd(control)

        return control
    

    
    def publish_cmd(self, control):
        # msg = WheelsCmdStamped()
        # msg.vel_left = self.speed + control
        # msg.vel_right = self.speed - control
        # self.wheel_publisher.publish(msg)

        #rospy.loginfo("Control: " + str(control))
        #rospy.loginfo("Error: " + str(self.error))

        self.test.append(control)
        message = Twist2DStamped(v=self.speed, omega= -1 * control)
        self.twisted_publisher.publish(message)

        pass
    

    def yellow_lane_callback(self, image):

        # Single line approach
        
        image = self._bridge.imgmsg_to_cv2(image) 

        # Creating contour to track red color 
        contours, hierarchy = cv.findContours(image, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE) 
        
        
        if len(contours)>0:
            # Sort contours by area in descending order and pick the top two
            max_contour = sorted(contours, key=cv.contourArea, reverse=True)[0]

            x_values = max_contour[:, 0, 0]  # Extracting x-coordinates

            # Compute the average x-coordinate
            avg_x = np.mean(x_values)
         
        # cv.line(image, (xc,yc), (xc+yc, yc), color=255)

        

        self.error = avg_x- image.shape[1]/2.0 + self.calibration

        # rospy.loginfo(self.error)

        # Contour approach

        return image
    
    def on_shutdown(self):
        # stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        # self.wheel_publisher.publish(stop)

        # rospy.loginfo(self.test)

        message = Twist2DStamped(v=0, omega=0)
        self.twisted_publisher.publish(message)
        
    def publish_velocity(self, **kwargs):
        # add your code here
        pass
        
    def stop(self, **kwargs):
        # add your code here
        msg = WheelsCmdStamped()
        msg.vel_left = 0
        msg.vel_right = 0
        self.wheel_publisher.publish(msg)

    def move_straight(self,  speed=0.5, direction=1, distance=0.3, calibrate = 1.3):
        msg = WheelsCmdStamped()
        msg.vel_left = speed * direction * 1.3
        msg.vel_right = speed * direction

        self.start_dist = self.compute_distance_traveled(self._ticks_left)

        while np.abs(self.start_dist - self.compute_distance_traveled(self._ticks_left)) < distance and not rospy.is_shutdown():
            self.wheel_publisher.publish(msg)

        msg.vel_left = 0
        msg.vel_right = 0
        self.wheel_publisher.publish(msg)
        pass
    
    def compute_distance_traveled(self, ticks):
        # left_distance = self._ticks_left* self.DISTANCE_PER_TICK
        # right_distance = self._ticks_right* self.DISTANCE_PER_TICK
        # dist = []
        # dist.append(left_distance)
        # dist.append(right_distance)
        distance = ticks* self.DISTANCE_PER_TICK
        return distance


    def start(self):
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            if self.imageFrame is None:
                continue
            # start = rospy.Time.now()

            # Blue line detection code
            self.color_detect_image = self.detect_blue_lane(self.imageFrame)
            # self.black_detect_image = self.detect_lane(self.disorted_image)
            # black_msg = self._bridge.cv2_to_imgmsg(self.color_detect_image, encoding="bgr8")
            # self.pub.publish(black_msg)
            # if self.blue_lane:
            #     rospy.loginfo("Blue lane detected")

            # Ducks detection code
            self.duck_detect_image = self.detect_ducks(self.imageFrame)
            # black_msg = self._bridge.cv2_to_imgmsg(self.duck_detect_image, encoding="bgr8")
            # self.pub.publish(black_msg)
            # if self.ducks:
            #     rospy.loginfo("Ducks detected")

            # PID control stuff
            self.black_detect_image = self.detect_lane(cv.blur(self.disorted_image, (5, 5)))
            # black_msg = self._bridge.cv2_to_imgmsg(self.black_detect_image, encoding="8UC1")
            # self.pub_augmented_image.publish(black_msg)
            # image_msg = self._bridge.cv2_to_imgmsg(self.gray, encoding="8UC1")
            # self.pub_augmented_image.publish(image_msg)
            # end = rospy.Time.now()
            # rospy.loginfo("Publish time: %f sec", (end - start).to_sec())

            # Motion Control
            if not self.blue_lane:
                self.get_control_output()  # Call control function continuously
                rospy.loginfo("Continue, Blue lane = False")
            else:
                self.stop()
                rospy.loginfo("Stop, Blue lane = True")
                if self.ducks:
                    rospy.loginfo("Sleep, Ducks = True")
                    rospy.sleep(1)
                else:
                    rospy.loginfo("Sleep, Blue lane = Change")
                    rospy.sleep(2)
                    self.move_straight()
                    self.blue_lane = False
            # rate.sleep()


if __name__ == '__main__':
    # create the node
    node = CrossWalkNode(node_name='april_tag_detector')
    node.start()
    rospy.spin()
