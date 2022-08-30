#!/usr/bin/env python
##THIS CODE SHOULD NOT WORK, USE AS OUTLINE
import rclpy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
# from rclpy.node import Node
#from std_msgs.msg import Float32
#from geometry_msgs.msg import Twist
#import time


#Node
NODE_NAME = 'color_detection_node'


# topics subscribed to
CAMERA_IMG_TOPIC_NAME = '/camera/color/image_raw'


## GOAL IS TO JUST TRACK COLOR AS OF RIGHT NOW


class TargetDetection(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        self.camera_subscriber = self.create_subscription(Image, CAMERA_IMG_TOPIC_NAME, self.camera_callback)
        self.bridge_object = CvBridge()


    def camera_callback(self,data): ##IMAGE SEEN FROM CAMERA
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)


        #Once we read the image we need to change the color space to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #Hsv limits are defined
        min_pink = np.array([150,50,50])
        max_pink = np.array([170,255,255])


        # mask and find contours
        # mask = cv2.inRange(hsv, blue_lower, blue_higher)
        mask = cv2.inRange(hsv, min_pink, max_pink)
        mask_pink = cv2.inRange(hsv, min_pink, max_pink)


        #We use the mask with the original image to get the colored post-processed image
        res_pink = cv2.bitwise_and(image, image, mask= mask_pink)

        cv2.imshow('Pink',res_pink)



def main():
    color_filter_object = ColorFilter()
    rospy.init_node('color_filter_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
