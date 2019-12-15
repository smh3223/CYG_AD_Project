import rospy
import cv2
import numpy as np
import pytesseract
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class numCheck:
    
    def __init__(self, topic):
    
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)
    
    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
    
    def detect_numbers(self):
        
        
        gray = cv2.cvtColor(self.cam_img, cv2.COLOR_BGR2GRAY)
        
        
        if cv2.waitKey(1) & 0xFF == 27:
            quit()    
        cv2.imshow("view", gray)
        
        ts = pytesseract.image_to_string(gray)
        print(ts)
        
        if 'SCHOOL' in ts:
            print('SCHOOL')
            return 'SCHOOL'
        elif 'BUS' in ts:
            print('BUS')
            return 'BUS'

