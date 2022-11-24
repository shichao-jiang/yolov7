import rospy
import os
from std_msgs.msg import Empty, UInt8, Bool
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
#from h264_image_transport.msg import H264Packet

from cv_bridge import CvBridge, CvBridgeError

#import av
import math
import numpy as np
import time
import torch

from djitellopy import Tello
from threading import Thread, Event
import time, cv2
import signal
import logging
from detect import detect_custom

bridge = CvBridge()

def runAlgorithm(image):
    print("Writing image...")
    image = bridge.imgmsg_to_cv2(image, 'rgb8')
    cv2.imwrite('tello/tello_image.jpg', image)
    detect_custom(source='tello/tello_image.jpg', imgsz=640, conf=0.25, save_img=True, project='tello/', name='exp', view_img=True)
    # cv2.imshow("YOLOv7 Output", image) # Display to window
    # os.remove('tello/tello_image.jpg')
    # cv2.waitKey(0)

def main():
    # create a  command subscriber instance
    print('ROS YOLOv7: Initializing ros_yolo node...')
    rospy.init_node('ros_yolo')
    rospy.Subscriber("tello/image_raw", Image, runAlgorithm, queue_size=1, buff_size=2**24)

    def exit_handler(signum, frame):
        msg = "Stopping YOLOv7"
        print(msg, end="", flush=True)
        exit(1)

    signal.signal(signal.SIGINT, exit_handler)

    print('ROS YOLOv7 wrapper is running, press Ctrl+C to terminate program.')
    rospy.spin()
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
