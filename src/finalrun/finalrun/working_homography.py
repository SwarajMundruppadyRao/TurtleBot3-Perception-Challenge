#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import time
from math import radians

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')


        #Define
        # self.subscription = self.create_subscription(CompressedImage,'/image_raw/compressed',self.image_callback,10)
        self.subscription = self.create_subscription(CompressedImage,'/camera/image_raw/compressed',self.image_callback,10)

        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.node = rclpy.create_node("a4sheet")
        self.index = 1
        self.a4_lst = []
        self.peak=0

    def image_callback(self, msg):
        # img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img=self.bridge.compressed_imgmsg_to_cv2(msg , "bgr8")
        #cv.imshow('Image', img)
        #cv.waitKey(1)
        # Apply Gaussian blur filter with a kernel size of 5x5
        blurred = cv.GaussianBlur(img, (3, 3), 0)

        # Convert image to grayscale
        gray = cv.cvtColor(blurred, cv.COLOR_BGR2GRAY)

        # Thresholding to get binary image
        _, binary_img = cv.threshold(gray, 180, 255, cv.THRESH_BINARY)
        cv.imshow('Thresholf', binary_img)
        cv.waitKey(1)
        
        img_c = np.copy(img)

        # Find contours
        contours, _ = cv.findContours(binary_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)        
        
        # Calculate the height of the image
        height, width, _ = img_c.shape

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
            msg = Twist()
            msg.linear.x = 0.0
            self.pub.publish(msg)
            msg = Twist()
            msg.linear.x = 0.0
            self.pub.publish(msg)
            #print("No contours detected")
            # Create a Twist message
            # Set the rotation speed to 15 degrees per second
            rotation_speed = radians(15)
            msg.angular.z = rotation_speed
                
            # Publish the twist message for 1 second to rotate the TurtleBot
            self.pub.publish(msg)
            time.sleep(1)
            # Stop the rotation
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return
        # else:
        #     flattened_contours = [point for contour in filtered_contours for point in contour]
        #     temp = np.array(flattened_contours, dtype=np.float32)
        #     approx = cv.approxPolyDP(temp, 0.01 * cv.arcLength(temp, True), True)
        #     # Check if the approximated contour has exactly four points
        #     print(len(approx))
            
        #     if len(approx) > 12:
        #         msg = Twist()
        #         msg.linear.x = 0.0
        #         self.pub.publish(msg)
        #         print("A4 SHEET NOT DETECTED")
        #         # Create a Twist message
        #         # Set the rotation speed to 15 degrees per second
        #         rotation_speed = radians(15)
        #         msg.angular.z = rotation_speed
                
        #         # Publish the twist message for 1 second to rotate the TurtleBot
        #         self.pub.publish(msg)
        #         time.sleep(1)
        #         # Stop the rotation
        #         msg.angular.z = 0.0
        #         self.pub.publish(msg)
        #         return    
                          

        
        # Assuming the largest contour is the A4 sheet
        a4_contour_max = max(filtered_contours, key=cv.contourArea)
        area = cv.contourArea(a4_contour_max)
        print(area)
        if(self.index < 21):
            self.a4_lst.append(area)
            self.index+=1
        else:
            max_value = max(self.a4_lst)
            index_elem = self.a4_lst.index(max_value)
            # print(index_elem)
            if(index_elem>=3 and index_elem<18):
                self.peak+=1
                # print("peak detected!!!!!!!!!!!!!!!!!!!!!!!!!", self.peak)
            self.a4_lst = []
            self.index = 1      

        if(area < 100):
            msg = Twist()
            msg.linear.x = 0.0
            self.pub.publish(msg)
            #print("A4 SHEET NOT DETECTED")
            # Create a Twist message
            # Set the rotation speed to 15 degrees per second
            rotation_speed = radians(30)
            msg.angular.z = rotation_speed
            
            # Publish the twist message for 1 second to rotate the TurtleBot
            self.pub.publish(msg)
            time.sleep(1)
            # Stop the rotation
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return  

        # Calculate the center of the largest contour
        M = cv.moments(a4_contour_max)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        
        # Update the center X position of the sheet
        sheet_center_x = cX
        
        # Calculate the error in X position relative to the center of the camera view
        width = img.shape[1]
        error = cX - width // 2
        msg = Twist()
        # Proportional control to move the TurtleBot
        kp = 0.002  # Proportional control constant
        angular_velocity = kp * error
        
        # Publish the velocity command to move the TurtleBot
        msg.linear.x = 0.05  # Forward speed
        msg.angular.z = -angular_velocity  # Angular velocity (turning)
        
        self.pub.publish(msg)
        
        # Optionally, display the processed image for debugging
        cv.drawContours(img_c, [a4_contour_max], -1, (0, 255, 0), 2)
        cv.imshow('Image', img_c)
        cv.waitKey(1)
        # if(self.peak >=8):
        #     msg = Twist()
        #     msg.linear.x = 0.0 # Forward speed
        #     msg.angular.z = 0.0
        #     self.pub.publish(msg)
        #     rclpy.shutdown()
        if cv.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main():
    print('Hi from enpm673_final_proj script.')
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
