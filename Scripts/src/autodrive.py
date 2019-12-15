import rospy, time

import numpy as np
from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver
from numCheck import numCheck
from imuread import ImuRead
from QRCodeCheck import QRCodeCheck
class AutoDrive:

    def __init__(self):
        self.Premid = 320
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.numCheck = numCheck('usb_cam/image_raw')
        self.QRCode = QRCodeCheck('usb_cam/image_raw')
        self.angle = 0
        self.isObs = False
        self.imu = ImuRead('/diagnostics')
        self.speed = 30
    def trace(self):
        global start_time
        
        print(self.speed)
        # imu part
        r, p, y = self.imu.get_data()
        #print('R (%.1f) P (%.1f), Y (%.1f)' % (r, p, y))
        speed= 30
        if p < -5:
            speed = 45
            self.driver.drive(90, 90 + speed)
            return 0
        elif p > 5:
            speed = -40
            self.driver.drive(90, 90)
            
            return 0
        
        
        #obs part
        
        
        obs_m = 1000
        if self.isObs == True:
            obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
            if obs_m < 30:
                obs_m = 1000
        else:
            obs_m = 1000
        
        
        line_l, line_r, theta = self.line_detector.detect_lines()
        WORD = self.numCheck.detect_numbers()
        QR = self.QRCode.detect_QR()
        self.line_detector.show_images(line_l, line_r)
        '''
        if time.time() - start_time > 20 and self.one == False:
            WORD = 'BUS'
            self.one = True
        '''
        # qr part
        
        if QR == '10':
            self.speed = 23
            print('10')
        
        elif QR == '20':
            self.speed = 30
            print('20')
        
        elif QR == '30':
            self.speed = 50

            print(30)
        
        elif QR == 'stop':
            time.sleep(30)

        elif QR == 'gorani':
            self.isObs = True
            self.speed = 30
            print('gorani')

        plus_angle = 2.2
        
        # word part
        
        if WORD == 'SCHOOL':
            self.speed = 20
            print('School')
        if WORD == 'BUS':
            time.sleep(1)
            speed = 20
            print('Bus')

        if obs_m > 70:
            if time.time() - start_time < 35:
                self.driver.drive(90+plus_angle, 90 + self.speed)
            else:
                self.driver.drive(90 + (theta * 180 / np.pi) * 1.4 + plus_angle, 90 + self.speed)
            
        else:
            self.driver.drive(90,90)
            self.isObs = False
            time.sleep(5)
            
            
    def steer(self, left, right):
        '''
        if left == -1 and right > -1:
            left = 720 - right
        elif left > -1 and right == -1:#
            right = 720 - lefts
        elif left == -1 and right == -1:
            left = 
        '''
        if left == -1 and right == -1:
            #self.angle *= 0.9
            return self.angle

        if left == -1 or right == -1:
            
            if left == -1:
                left = right - 480
            else:
                right = left + 480

        mid = (left + right) // 2 - (270 - 320) 
        if True or right - left >= 380:
                
            if mid < 315:
                angle = -50
            elif mid > 440:
                angle = 50
            else:
                angle = 0
        else:
            if mid > 300:
                angle = -20
            else:
                angle = 20
            if left < 320 and right > 320:
                angle *= -1
        
        self.angle = angle
        return self.angle
    def accelerate(self, angle, left, mid, right):
        #if min(left, mid, right) < 50:
        #    speed = 0
        if angle < -20 or angle > 20: #elif angle < -20 or angle > 20:
            speed = 30
        else:
            speed = 45
        return speed

    def exit(self):
        print('finished')

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    start_time = time.time()
    
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)

