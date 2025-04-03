#!/usr/bin/env python3

# potentially useful for question - 1.5

# import required libraries
import rospy
from duckietown.dtros import DTROS, NodeType

import os
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
import numpy as np
from duckietown_msgs.msg import LEDPattern, WheelEncoderStamped

import cv2 as cv
from cv_bridge import CvBridge
import dt_apriltags as aptag
from std_msgs.msg import Header, ColorRGBA, Int32, String

class NavigationControl(DTROS):
    def __init__(self, node_name):
        super(NavigationControl, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here
        
        # publisher for wheel commands
        # NOTE: you can directly publish to wheel chassis using the car_cmd_switch_node topic in this assignment (check documentation)
        # you can also use your exercise 2 code
        
        # robot params

        # define other variables as needed
        # controller type
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
        # self.wheel_publisher = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)

        self.lane_topic = f"/{self._vehicle_name}/custom_node/image/black"

        self.lane_sub = rospy.Subscriber(self.lane_topic, Image, self.yellow_lane_callback)

        # yellow bound
        self.yellow_lower = np.array([20, 80, 100], np.uint8) 
        self.yellow_upper = np.array([40, 255, 255], np.uint8) 

        # while bound
        self.white_lower = np.array([0, 0, 180], np.uint8) 
        self.white_upper = np.array([180, 40, 255], np.uint8)

        # Set range for red color
        self.red_lower = np.array([130, 70, 80], np.uint8) 
        self.red_upper = np.array([190, 255, 255], np.uint8)


        self._bridge = CvBridge()

        # twisted topic
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        self._v = 0
        self._omega = 0
        # construct publisher
        self.twisted_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)


        # None lane-following stuff

        # self.line_disappear = False
        self.red_lane_message = "No"

        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self._string_topic = f"/{self._vehicle_name}/control_node/red_lane"

        self._ticks_left = 0
        self._ticks_right = 0
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.sub_instruction = rospy.Subscriber(self._string_topic, String, self.callback_string)
        self.pub = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)
        
        # publisher for wheel commands
        # NOTE: you can directly publish to wheel chassis using the car_cmd_switch_node topic in this assignment (check documentation)
        # you can also use your exercise 2 code
        
        # robot params
        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)
        self.DISTANCE_PER_TICK = np.pi*2*self._radius/135
        self.start_dist = 0

        self._apriltag_topic = f"/{self._vehicle_name}/control_node/apriltag"
        self.sub_april = rospy.Subscriber(self._apriltag_topic, Int32, self.callback_apriltag)
        self.aprilid = 1000
    
    def callback_left(self, data):
        self._ticks_left = data.data

    def callback_right(self, data):
        self._ticks_right = data.data
    
    def callback_apriltag(self, data):
        aid = data.data
        # rospy.loginfo(aid)
        if aid != 1000:
            self.aprilid = aid
    
    def callback_string(self, data):
        rate = rospy.Rate(10)
        self.red_lane_message = data.data
        rospy.loginfo(self.red_lane_message)

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
    
    def compute_distance_traveled(self, ticks):
        # left_distance = self._ticks_left* self.DISTANCE_PER_TICK
        # right_distance = self._ticks_right* self.DISTANCE_PER_TICK
        # dist = []
        # dist.append(left_distance)
        # dist.append(right_distance)
        distance = ticks* self.DISTANCE_PER_TICK
        return distance
    
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
    
    def detect_line(self, imageFrame):
        hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)
        # red mask
        # rospy.loginfo("line detecting")
        if self.color == 'r':
            mask = cv.inRange(hsvFrame, self.red_lower, self.red_upper) 
        elif self.color == "g":
            mask = cv.inRange(hsvFrame, self.green_lower, self.green_upper)
        elif self.color == "b":
            mask = cv.inRange(hsvFrame, self.blue_lower, self.blue_upper)

        kernel = np.ones((5, 5), "uint8") 
        mask = cv.dilate(mask, kernel) 
        res = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = mask) 
        contours, hierarchy = cv.findContours(mask, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours): 
            area = cv.contourArea(contour) 
            if(area > 100): 
                x, y, w, h = cv.boundingRect(contour) 
                rospy.loginfo(y+h)
                # imageFrame = cv.rectangle(imageFrame, (x, y), 
                #                         (x + w, y + h), 
                #                         (0, 0, 255), 2) 
                
                # cv.putText(imageFrame, "Colour", (x, y), 
                #             cv.FONT_HERSHEY_SIMPLEX, 1.0, 
                #             (0, 0, 255))
                if y + h > 110:
                    self.line_disappear = True
                    rospy.loginfo("Stop")
        return imageFrame
    
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
        self.pub.publish(msg)
        
    def move_straight(self,  speed=0.5, direction=1, distance=0.3, calibrate = 1.3):
        msg = WheelsCmdStamped()
        msg.vel_left = speed * direction * 1.3
        msg.vel_right = speed * direction

        self.start_dist = self.compute_distance_traveled(self._ticks_left)

        while np.abs(self.start_dist - self.compute_distance_traveled(self._ticks_left)) < distance and not rospy.is_shutdown():
            self.pub.publish(msg)

        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        pass
        
    def turn_right(self, **kwargs):
        # add your code here
        pass
        
    def turn_left(self, **kwargs):
        # add your code here
        pass

    def start(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.red_lane_message == "No":
                self.get_control_output()  # Call control function continuously
            else:
                self.red_lane_message = "No"
                rospy.loginfo(self.aprilid)
                self.stop()
                if self.aprilid == 50 or self.aprilid == 133:
                    # T Intersetion Tag
                    rospy.sleep(2)
                    self.aprilid == 1000
                elif self.aprilid == 22 or self.aprilid == 21:
                    # Stop sign
                    rospy.sleep(3)
                    self.aprilid == 1000
                elif self.aprilid == 93 or self.aprilid == 94:
                    # U of A sign
                    rospy.sleep(1)
                    self.aprilid == 1000
                else:
                    rospy.sleep(0.5)
                self.move_straight()
            rate.sleep()


    # add other functions as needed

if __name__ == '__main__':
    node = NavigationControl(node_name='navigation_control_node')
    # node.start()
    rate = rospy.Rate(10)  # 10 Hz
    node.start()

    # while not rospy.is_shutdown():
    #     if node.red_lane_message == "No":
    #         node.get_control_output()  # Call control function continuously
    #     else:
    #         node.red_lane_message = "No"
    #         rospy.loginfo("Stop")
    #         node.stop()
    #         rospy.sleep(2)
    #         node.move_straight()

    #     rate.sleep()
    rospy.spin()