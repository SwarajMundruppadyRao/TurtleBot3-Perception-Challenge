>#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import time
from math import radians
import sys
from std_msgs.msg import Bool


#defines
K_P = 0.002
ROTATION_ANGLE = 15

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        #Reading the compressed image
        self.subscription = self.create_subscription(CompressedImage,'/camera/image_raw/compressed',self.image_callback,10)
        # Subscribers for stop sign and moving object detection
        self.stop_sub = self.create_subscription(
            Bool, '/stop_sign_detected', self.stop_callback, 10)
        self.moving_sub = self.create_subscription(
            Bool, '/optical_flow', self.moving_callback, 10)
        
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.node = rclpy.create_node("a4sheet")
        self.index = 1
        self.a4_lst = []
        self.peak=0
        self.stop_sign = False
        self.dynamic = False
        self.twist = Twist()
        self.kp = K_P
        self.rotation_speed = radians(ROTATION_ANGLE)
        self.counter=0
        self.TURN=0

    def stop_callback(self, msg):
        # Print the received stop sign detection status only if True
        if msg.data == True:
            self.get_logger().info('Stop sign detected.')
            self.stop_sign = True
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)  
            time.sleep(1)         

        else:
            self.stop_sign = False
            self.get_logger().info('Stop sign NOT detected.')

    def moving_callback(self, msg):
        # Print the received moving object detection status only if True
        if msg.data == True:
            self.get_logger().info('Moving object detected.')
            self.dynamic = True
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)
            time.sleep(1)
        else:
            self.dynamic = False
            self.get_logger().info('Moving object NOT detected.')

    def image_callback(self, msg):
        
        #Added this stop the code in simulation environment, can change the value as desired
        if self.TURN==5:
            print("TURN= ", self.TURN)  
            self.twist.linear.x = 0.05
            time.sleep(5)
            self.pub.publish(self.twist)
            self.twist.linear.x = 0.0
            self.pub.publish(self.twist)
            rclpy.shutdown()

        if((self.stop_sign ==  False) and (self.dynamic == False)):
            start_time = time.time() 
            #Read the compressed image
            img=self.bridge.compressed_imgmsg_to_cv2(msg , 'bgr8')
            
            #apply Gaussian blur filter with a kernel size of 3x3
            blurred = cv.GaussianBlur(img, (3, 3), 0)

            # Convert image to grayscale
            gray = cv.cvtColor(blurred, cv.COLOR_BGR2GRAY)
            

            # Thresholding to get binary image
            _, binary_img = cv.threshold(gray, 180, 255, cv.THRESH_BINARY)
            
            img_c = np.copy(img)      
           
            # Calculate the height of the image
            height, width, _ = img_c.shape

            # Find contours    
            contours, _ = cv.findContours(binary_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            # Calculate the half height of the image
            half_height = height / 2

            # List to hold filtered contours
            filtered_contours = []

            # Filter out contours below half the height
            for contour in contours:
            # Get the bounding box of the contour
                x, y, w, h = cv.boundingRect(contour)

                # Check if the bottom of the bounding box is above half of the image height
                if y >= half_height:
                # If the contour is completely above the half height, add it to the list of filtered contours
                    filtered_contours.append(contour)
            
            if len(filtered_contours)==0:
                #Rotate the robot by 15 degrees if no contours are detected
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.rotation_speed
                self.TURN+=1                   
                self.pub.publish(self.twist)
                
                time.sleep(1)

                # Stop the rotation
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.pub.publish(self.twist)
                return                  
            
            # Assuming the largest contour is the A4 sheet
            a4_contour_max = max(filtered_contours, key=cv.contourArea)
            area = cv.contourArea(a4_contour_max)    

            if(area < 100):
                
                #Rotate the robot by 15 degrees if area of the contours detected are less than 100
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.rotation_speed
                self.TURN+=1
                self.pub.publish(self.twist)

                time.sleep(1)

                # Stop the rotation
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.pub.publish(self.twist)
                return  

            # Calculate the center of the largest contour
            M = cv.moments(a4_contour_max)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
                       
            # Calculate the error in X position relative to the center of the camera view
            width = img.shape[1]
            error = cX - width // 2

            #Propotional control to move the robot
            angular_velocity = self.kp * error
            
            # Publish the velocity command to move the TurtleBot
            self.twist.linear.x = 0.05 # Forward speed
            self.twist.angular.z = -angular_velocity  # Angular velocity (turning)
            
            self.pub.publish(self.twist)

            end_time = time.time()    # Record the end time

            elapsed_time = end_time - start_time  # Calculate elapsed time

            print(f"Homography took {elapsed_time:.4f} seconds to run.")

            print("TURN= ", self.TURN)


def main():
    print('Hi from enpm673_final_proj script.')
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()