#!/usr/bin/env python3
import sys
import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Int32

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        #subcribing to the image published
        self.subscription = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)
        #create a publisher 
        self.horizon_pub = self.create_publisher(Int32, '/horizon_line', 10)
        self.bridge = CvBridge()
        self.frame_count = 0  
        self.skip_frames = 10  #frames to skip before processing
        self.processed_first_frame = False
        self.has_detected=False
        self.detected_value=None

    def image_callback(self, msg):
        if self.has_detected:
            return
        if self.frame_count < self.skip_frames:
            self.frame_count += 1
            return  # Skip processing if the first frame has been processed

        # getting the first frame for processing
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        # converting to gray scale
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        #getting the paper only
        _, mask = cv.threshold(gray, 210, 255, cv.THRESH_BINARY)
        #bluring the frame
        gray_blurred = cv.GaussianBlur(mask, (3, 3), 0)
        #edge detection
        edges = cv.Canny(gray_blurred, 160, 140)
        # line detection
        lines = cv.HoughLinesP(edges, 1, np.pi/180, 13, minLineLength=140, maxLineGap=10)

        if lines is None:
            print("No lines detected")
            return

        slopes = []
        #calculating the line equations
        if lines is not None:
            for line_1 in lines:
                x_1,y_1,x_2,y_2 = line_1[0]
                cv.line(img, (x_1, y_1), (x_2, y_2), (0, 255, 0), 2)

                m = (y_2 - y_1)/(x_2 - x_1)
                y_inter = y_1 - m*x_1
                slopes.append([m,y_inter])
    
        if not slopes:
            print("No valid lines for processing")
            return

        filtered_slopes = []
        #filtering the slopes and removing the lines that lie one over the other
        while slopes:
            current = slopes.pop(0)  #first line to compare with others
            is_unique = True
            for other in slopes:
                if current[0] == np.inf and other[0] == np.inf and abs(current[1] - other[1]) < 10:
                    is_unique = False
                    break
                elif abs(current[0] - other[0]) < 0.01 and abs(current[1] - other[1]) < 10:
                    is_unique = False
                    break
            if is_unique:
                filtered_slopes.append(current)

        #filter out the lines to get the correct slopes and parallel lines lined up
        filtered_slopes = sorted(filtered_slopes, key=lambda x: x[0])
        #calculate the intersections
        intersections = []
        for i in range(0,len(filtered_slopes)-1,2):
            intersect = self.calculate_intersections(filtered_slopes[i][0], filtered_slopes[i][1], filtered_slopes[i+1][0], filtered_slopes[i+1][1])
            if intersect[0] is not None:
                intersections.append(intersect)
      
        

        if intersections:
            average_value = sum(y for _, y in intersections) / len(intersections)  # Calculate the average y-coordinate
            average_y = Int32()  
            average_y.data = int(average_value) 
            self.detected_value=average_y.data
            # publish the y value and show the line 
            self.has_detected=True
            cv.line(img, (0, int(average_value)), (img.shape[1], int(average_value)), (0, 0, 255), 2)
            cv.imshow('horizon.jpg', img)
            key = cv.waitKey(1)
            if key & 0xFF == ord('q'):
                cv.destroyAllWindows()
                sys.exit(0)


        self.processed_first_frame = True  #flag to true after processing

    def publish_detected_value(self):
        if self.detected_value is not None:
            average_y = Int32()
            average_y.data = int(self.detected_value)
            self.horizon_pub.publish(average_y)
            print(average_y)

    #calculate the intersection
    def calculate_intersections(self, m1, b1, m2, b2):
        
        if abs(m1 - m2) < 1e-5:
            return (None, None)  #if lines are parallel
        x_intersect = (b2 - b1) / (m1 - m2)
        y_intersect = m1 * x_intersect + b1
        return (x_intersect, y_intersect)
    
def main():
    print('Starting Horizon Line Code.')
    rclpy.init()
    image_subscriber = ImageSubscriber()
    while rclpy.ok():
        rclpy.spin_once(image_subscriber, timeout_sec=1)
        image_subscriber.publish_detected_value()
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()