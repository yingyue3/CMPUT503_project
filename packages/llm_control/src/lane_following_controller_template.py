#!/usr/bin/env python3

# potentially useful for question - 2.2

# import required libraries

import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
import os
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge

class LaneControllerNode(DTROS):
    def __init__(self, node_name):
        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here
        
        # controller type
        self.control_type = "PID"  # it can be P or PD or PID
        self.English_driver = False

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
        self.speed = 0.35
        # self.speed = 0
        self.error = 0
        
        # distance tracking
        #self.calibration = 130

        self.calibration = 109

        self.test = []
        
        # initialize publisher/subscribers
        self.vehicle_name = os.environ['VEHICLE_NAME']
        self.wheels_topic = f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd"
        self.wheel_publisher = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)

        self.lane_topic = f"/{self.vehicle_name}/custom_node/image/black"

        self.lane_sub = rospy.Subscriber(self.lane_topic, Image, self.yellow_lane_callback)

        # yellow bound
        self.yellow_lower = np.array([20, 80, 100], np.uint8) 
        self.yellow_upper = np.array([40, 255, 255], np.uint8) 

        # while bound
        self.white_lower = np.array([0, 0, 180], np.uint8) 
        self.white_upper = np.array([180, 40, 255], np.uint8)

        self._custom_topic_lane = f"/{self.vehicle_name}/custom_node/image/test"
        self.pub_lane = rospy.Publisher(self._custom_topic_lane, Image, queue_size=1 )

        self._bridge = CvBridge()

        # twisted topic
        twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        self._v = 0
        self._omega = 0
        # construct publisher
        self.twisted_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)


    

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
        

    def non_maximum_suppression(self, sobel, ksize=3):
        
        h, w = sobel.shape
        suppressed = np.zeros_like(sobel)  # Output image with suppressed values

        half_k = ksize // 2  # Half window size for indexing

        for i in range(half_k, h - half_k):
            for j in range(half_k, w - half_k):
                local_window = sobel[i - half_k:i + half_k + 1, j - half_k:j + half_k + 1]
                max_val = np.max(local_window)

                # Keep only the pixel that holds the max value in the window
                if sobel[i, j] == max_val:
                    suppressed[i, j] = sobel[i, j]

        return suppressed

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

        '''white = np.zeros_like(image)
        white[image == 255] = 255
        
        yellow = np.zeros_like(image)
        yellow[image == 128] = 255
        # Creating contour to track red color 
        contours, hierarchy = cv.findContours(white, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE) 
        
        
        if len(contours)>0:
            # Sort contours by area in descending order and pick the top two
            max_contours = sorted(contours, key=cv.contourArea, reverse=True)[0]

            # Get bounding boxes for both contours
            x1, y1, w1, h1 = cv.boundingRect(max_contours)
            
        contours, hierarchy = cv.findContours(yellow, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE) 
        
        
        if len(contours)>0:
            # Sort contours by area in descending order and pick the top two
            max_contours = sorted(contours, key=cv.contourArea, reverse=True)[0]

            # Get bounding boxes for both contours
            x2, y2, w2, h2 = cv.boundingRect(max_contours)

            
        
        w = (w1//h1)*(h1 - (y2+h2//2-y1))
        cv.circle(image, (x1+w, y2+h2//2), radius=5, color=180, thickness=-1)
        cv.circle(image, (x2+w2//2, y2+h2//2), radius=5, color=180, thickness=-1)

        # cv.line(image, (xc,yc), (xc+yc, yc), color=255)

        msg = self._bridge.cv2_to_imgmsg(image, encoding="8UC1")
        
        self.pub_lane.publish(msg)

        self.error = (x1 + x2)/2.0 - image.shape[1]/2.0'''

        # Two line approach


        '''h, w = image.shape
        image = image[h//3:-h//3, :]
 
        sobely = cv.Sobel(image, cv.CV_64F, 0, 1, ksize=3)  # 1 in X direction, 0 in Y

        # Convert to absolute values and normalize to 0-255
        sobely = np.absolute(sobely)
        sobely = np.uint8(255 * sobely / np.max(sobely)) 
        #sobely = self.non_maximum_suppression(sobely, ksize=3)
                
        # black_msg = self._bridge.cv2_to_imgmsg(sobely, encoding="8UC1")
        
        # self.pub_lane.publish(black_msg)

        prev_error = self.error

        temp_error = 0
        count = 0

        if not self.English_driver:


            rows_with_yellow = np.where(np.any(sobely == 128, axis=1))[0]

            if len(rows_with_yellow) > 0:  # Ensure value exists in the array
                row = rows_with_yellow[0]  # Get the first row where it appears
                yellow_pixels = np.where(sobely[row, :] == 128)[0]

                last_yellow = yellow_pixels[-1]

                white_pixels = np.where(sobely[row, last_yellow:] > 200)[0]
                # rospy.loginfo(white_pixels)

                if(len(white_pixels) != 0):
                    count +=1
                
                    first_white = white_pixels[0] + last_yellow
                    temp_error+= ((int(first_white) + int(last_yellow))//2 - image.shape[1]//2) 
        else: 
            rows_with_yellow = np.where(np.any(sobely == 128, axis=1))[0]

            if len(rows_with_yellow) > 0:  # Ensure value exists in the array
                row = rows_with_yellow[0]  # Get the first row where it appears
                yellow_pixels = np.where(sobely[row, :] == 128)[0]

                first_yellow = yellow_pixels[0]

                white_pixels = np.where(sobely[row, :first_yellow] > 200)[0]
                # rospy.loginfo(white_pixels)

                if(len(white_pixels) != 0):
                    count +=1
                
                    last_white = white_pixels[-1] 
                    temp_error+= ((int(last_white) + int(first_yellow))//2 - image.shape[1]//2)

                    # rospy.loginfo("First yellow:" + str(first_yellow))
                    # rospy.loginfo("Last white:" + str(last_white))


        # for row in range(image.shape[0]):
        #     # rospy.loginfo(sobely[row, :])
        #     yellow_pixels = []
        #     yellow_pixels = np.where((sobely[row, :] > 110) & (sobely[row, :] < 140))[0]

        #     if(len(yellow_pixels) != 0):
        #         # rospy.loginfo(yellow_pixels)
                
        #         last_yellow = yellow_pixels[-1]

        #         white_pixels = np.where(sobely[row, last_yellow:] > 200)[0]
        #         # rospy.loginfo(white_pixels)

        #         if(len(white_pixels) != 0):
        #             count +=1
                
        #             first_white = white_pixels[0] + last_yellow
        #             temp_error+= ((int(first_white) + int(last_yellow))//2 - image.shape[1]//2) 
        #             break

        #             # rospy.loginfo("Last yellow: " + str(last_yellow))
        #             # rospy.loginfo("First white: " + str(first_white))
        #             # rospy.loginfo("Middle: " + str(image.shape[1]//2))

        # if count == 0:
        #     self.speed = 0
        # if count != 0:

        #     self.error = temp_error // count 
        # else:
        self.error = temp_error'''

        return image
    

    # add other functions as needed
    def on_shutdown(self):
        # stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        # self.wheel_publisher.publish(stop)

        rospy.loginfo(self.test)

        message = Twist2DStamped(v=0, omega=0)
        self.twisted_publisher.publish(message)

if __name__ == '__main__':
    node = LaneControllerNode(node_name='lane_controller_node')
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        node.get_control_output()  # Call control function continuously
        rate.sleep()