#!/usr/bin/env python3
import sys
import time
import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from ultralytics import YOLO
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        #Create a subscriber to the camera image
        self.subscription = self.create_subscription(CompressedImage,'/camera/image_raw/compressed',self.image_callback,10)
        self.subscription  # prevent unused variable warning
        
        #Create a bridge to convert the image from ROS to OpenCV
        self.bridge = CvBridge()
        
        #Create a publisher to publish the velocity commands
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #Create a publisher to publish the initial pose
        self.pub_reset = self.create_publisher(Pose, '/initial_pose', 10)
        
        #Create a flag to check if the stop sign is detected
        self.stop_detected_time = None
        
        #Create a Twist object to store the velocity commands
        self.move_cmd = Twist()
        
        #Threshold for the stop sign detection in YOLO
        self.threshold = 0.4
        
        #Load the YOLO model which is trained on the Stop Sign dataset
        self.model=YOLO("/home/swaraj/ENPM673_turtlebot_perception_challenge/src/finalrun/finalrun/best.pt")
        
        #Create a publisher to publish the stop sign detection
        self.stop_sign=self.create_publisher(Bool, '/stop_sign_detected', 10)


    def image_callback(self, msg):
        #Read the image from the camera 
        img= self.bridge.compressed_imgmsg_to_cv2(msg)
        
        #Detect the stop sign in the image
        results = self.model(img)[0]
        
        #Draw the bounding box around the stop sign if detected
        detected = False
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result
            
            #Check if the detected object is a stop sign and the confidence score is greater than the threshold
            if score > self.threshold: 
                
                #Change the flag to True if the stop sign is detected
                detected=True
                print("Stop sign detected")
                
                #Draw the bounding box around the stop sign
                cv.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                
                #Display the label and the confidence score
                label = results.names[int(class_id)].upper()
                label_text = f"{label} {score:.2f}"
                cv.putText(img, label_text, (int(x1), int(y1 - 10)), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                
                #Publish the stop sign detection
                detected = True
            self.stop_sign.publish(Bool(data=detected))
                

        
        #Display the image with the bounding box in the cv window
        cv.imshow('YOLO Object Detection', img)
        key = cv.waitKey(1)
        if key == ord('q'):
            cv.destroyAllWindows()
            sys.exit()

def main():
    print('Hi from Stop Sign Detection Script.')
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()