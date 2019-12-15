import pyzbar.pyzbar as pyzbar
import cv2
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class QRCodeCheck:
    def __init__(self, topic):
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)
    
    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
    
    def detect_QR(self):
        img = self.cam_img
        plt.imshow(img)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        plt.imshow(gray, cmap='gray')

        decoded = pyzbar.decode(gray)

        for d in decoded:
            print(d.data.decode('utf-8'))
            print(d.type)
            
            return str(d.data.decode('utf-8')) 
            cv2.rectangle(img, (d.rect[0], d.rect[1]), (d.rect[0] + d.rect[2], d.rect[1] + d.rect[3]), (0, 0, 255), 2)
           
        plt.imshow(img)
