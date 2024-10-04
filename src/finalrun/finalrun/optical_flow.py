#!/usr/bin/env python3
import sys
import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import math

class ImageSubscriber(Node):
    def __init__(self):
        #publisher and subscriber initialization
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.prev_gray = None
        self.hsv = None
        self.frame_count = 0
        self.twist = Twist()
        self.optical_flow_pub=self.create_publisher(Bool, '/optical_flow', 10)
        self.horizon_sub = self.create_subscription(Int32, '/horizon_line', self.horizon_callback, 10)
        self.horizon_y=None
        self.count=0
    #calback function for horizontal lines
    def horizon_callback(self, msg):
        self.horizon_y=msg.data
    #main processing
    def image_flow(self, frame):
        self.detected_optical = False
        print("Image Flow")
        # print(self.horizon_y)
        # getting consecutive frames
        if self.prev_gray is None:
            self.prev_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            self.hsv = np.zeros_like(frame)
            #converting to hsv
            self.hsv[..., 1] = 255
            h,w,_ = frame.shape
        next_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        #using farenback algorithm
        flow = cv.calcOpticalFlowFarneback(self.prev_gray, next_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        # getting the magnitude and direction
        magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])
        angle_degrees = angle * 180 / np.pi
        self.hsv[..., 0] = angle * 180 / np.pi / 2
        self.hsv[..., 2] = cv.normalize(magnitude, None, 0, 255, cv.NORM_MINMAX)

        rgb = cv.cvtColor(self.hsv, cv.COLOR_HSV2BGR)
        cv.imshow('Dense Optical Flow', rgb)
        #setting threshhold fro contour generation
        motion_threshold = 20
        angle_threshold = 15

        #seperating horizontal and vertical motion
        horizontal_mask = ((angle_degrees < angle_threshold) | (angle_degrees > 360 - angle_threshold)) | \
                          ((angle_degrees > 180 - angle_threshold) & (angle_degrees < 180 + angle_threshold))

        vertical_mask = (angle_degrees > 45) & (angle_degrees < 135) | (angle_degrees > 225) & (angle_degrees < 315)
        #initializing mask
        motion_mask = (magnitude > motion_threshold) & horizontal_mask & ~vertical_mask

        motion_speed = np.mean(magnitude[motion_mask])
        print("Motion Speed: ", motion_speed)
        contours, _ = cv.findContours(motion_mask.astype(np.uint8), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            c_area = cv.contourArea(contour)
            # print("Contour Area: ", c_area)
            if cv.contourArea(contour) > 1000: 
                #generating the bounding boxes
                x, y, w, h = cv.boundingRect(contour)
                cv.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                bounding_area=(x+w)*(y+h)
                print("Bounding Area: ", bounding_area)
                if self.horizon_y is not None and not math.isnan(y) and int(y+(h/2)) >= int(self.horizon_y):
                    print("Centroid of Motion Detected object in the lower half of the image.")
                    self.get_logger().info('Centroid of Motion Detected object is below the horizon line.')
                    # self.detected_optical = True
                else:
                    print("Centroid of Motion Detected object is above the horizon line.")
                    # self.detected_optical = False
                #publishing true if the object is moving
                if motion_speed < 12 and bounding_area < 200:
                    self.detected_optical = False
                else:
                    self.detected_optical = True
        
        width = frame.shape[1]
        #publishing the value
        self.optical_flow_pub.publish(Bool(data=self.detected_optical))
        cv.line(frame, (0, self.horizon_y), (width, self.horizon_y), (0, 255, 0), 2)
        cv.putText(frame, "Horizon Line", (0, self.horizon_y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv.imshow('Motion Detection', frame)

        self.prev_gray = next_gray
    # gettting the frames
    def image_callback(self, msg):
        frame=self.bridge.compressed_imgmsg_to_cv2(msg)
        self.image_flow(frame)
        if cv.waitKey(1) & 0xFF == ord('q'):  
            cv.destroyAllWindows()
            sys.exit(0)

            return

def main():
    print('Starting Optical Flow Code.')
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()