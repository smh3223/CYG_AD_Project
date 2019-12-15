import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from numpy import ones,vstack
from numpy.linalg import lstsq



class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.value_threshold = 200
        self.image_width = 640
        self.scan_width, self.scan_height = 280, 30
        self.lmid, self.rmid = self.scan_width, self.image_width - self.scan_width
        self.area_width, self.area_height = 20, 10
        self.roi_vertical_pos = 300
        self.row_begin = 35#(self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height
        self.pixel_cnt_threshold = 0.3 * self.area_width * self.area_height

        
        
        
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        '''
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        '''
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)

    def roi(self,img, vertices):
        mask = np.zeros_like(img)
        if len(img.shape) > 2:
            ignore_mask_color = (255,) * image.shape[2]
        else:
            ignore_mask_color = 255

        cv2.fillPoly(mask, vertices, 255)

        masked = cv2.bitwise_and(img, mask)
        return masked


    def draw_lanes(self,img, lines, color=[0, 255, 255], thickness=3):
        
        ys = []
        for i in lines:
            for ii in i:
                ys += [ii[1], ii[3]]
        min_y = min(ys)
        max_y = 600
        new_lines = []
        line_dict = {}

        for idx, i in enumerate(lines):
            for xyxy in i:
                x_coords = (xyxy[0], xyxy[2])
                y_coords = (xyxy[1], xyxy[3])
                A = vstack([x_coords, ones(len(x_coords))]).T
                m, b = lstsq(A, y_coords)[0]

                x1 = (min_y - b) / m
                x2 = (max_y - b) / m

                line_dict[idx] = [m, b, [int(x1), min_y, int(x2), max_y]]
                new_lines.append([int(x1), min_y, int(x2), max_y])

        final_lanes = {}

        for idx in line_dict:
            final_lanes_copy = final_lanes.copy()
            m = line_dict[idx][0]
            b = line_dict[idx][1]
            line = line_dict[idx][2]

            if len(final_lanes) == 0:
                final_lanes[m] = [[m, b, line]]

            else:
                found_copy = False

                for other_ms in final_lanes_copy:

                    if not found_copy:
                        if abs(other_ms * 1.2) > abs(m) > abs(other_ms * 0.8):
                            if abs(final_lanes_copy[other_ms][0][1] * 1.2) > abs(b) > abs(
                                            final_lanes_copy[other_ms][0][1] * 0.8):
                                final_lanes[other_ms].append([m, b, line])
                                found_copy = True
                                break
                        else:
                            final_lanes[m] = [[m, b, lidne]]

        line_counter = {}

        for lanes in final_lanes:
            line_counter[lanes] = len(final_lanes[lanes])

        top_lanes = sorted(line_counter.items(), key=lambda item: item[1])[::-1][:2]

        lane1_id = top_lanes[0][0]
        lane2_id = top_lanes[1][0]

        def average_lane(lane_data):
            dat = [[] for _ in range(4)]
            for data in lane_data:
                for i in range(4):
                    dat[i].append(data[2][i])
            return map(lambda x: int(cv2.mean(np.array(x))[0]),
                       dat)  # int(cv2.mean(x1s)), int(cv2.mean(y1s)), int(cv2.mean(x2s)), int(cv2.mean(y2s))

        l1_x1, l1_y1, l1_x2, l1_y2 = average_lane(final_lanes[lane1_id])
        l2_x1, l2_y1, l2_x2, l2_y2 = average_lane(final_lanes[lane2_id])

        return [l1_x1, l1_y1, l1_x2, l1_y2], [l2_x1, l2_y1, l2_x2, l2_y2], lane1_id, lane2_id


    def process_img(self,image):
        if len(np.shape(image)) != 3:
            image = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        image = image[:]
        #image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        original_image = image[:]

        processed_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        edge = processed_img = cv2.Canny(processed_img, threshold1=50, threshold2=300)

        processed_img = cv2.Canny(processed_img, threshold1 = 50, threshold2=300)


        height, width = 200,200

        srcPoint=np.array([[0, 319], [173, 257], [460, 257], [637, 324]], dtype=np.float32)
        dstPoint=np.array([[0, height], [0, 0], [width, 0], [width, height]], dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(srcPoint, dstPoint)

        dst = cv2.warpPerspective(edge, matrix, (width, height))
        dst_debug = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        
        lines = cv2.HoughLines(dst,1,np.pi/180, 95)
        theta = 0
        rho = 0
        aa = []
        if lines is not None:
            thetas = 0
            rhos = 0
            for i in xrange(len(lines)):
                for rho, theta in lines[i]:
                    if theta > np.pi / 2:
                        theta = theta - np.pi
                    
                    #print(theta)
                    if (theta * 180 / np.pi) > -2 and (theta * 180 / np.pi < 2):
                        theta = 0
                    #print(theta)
                    if theta > -1.5 and theta < 1.5:
		                thetas = thetas + theta
		                aa += [theta]
		                rhos = rhos + rho
                    
                    #print(theta)
                    #print(theta * 180 / np.pi)
                    '''
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0+1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 -1000*(a))

                    cv2.line(dst_debug,(x1,y1),(x2,y2),(0,0,255),2)
                    '''
            #print(thetas)
            
            plus = 0
            minus = 0
            for i in aa:
	            if i > 0:
	                plus += 1
	            if i < 0:
	                minus += 1
            bb = []
            if plus > 8 and minus > 5:
                for i in aa:
                    if i < 0:
                        bb.append(i)
                aa = bb
            
            if len(aa) != 0:
                thetas = sum(aa)
                theta = thetas / len(aa)
                rho = rhos / len(aa)
            else:
                theta = thetas / len(lines)
                rho = rhos / len(lines)
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0+1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 -1000*(a))

        cv2.line(dst_debug,(x1,y1),(x2,y2),(0,0,255),2)
        
         

        #cv2.imshow("dst", dst_debug)
        
        #processed_img = cv2.GaussianBlur(processed_img,(3,3),0)
        processed_img = cv2.morphologyEx(processed_img, cv2.MORPH_CLOSE, np.ones((5, 5)))
        
        vertices = np.array([[10,500],[10,300],[300,200],[500,200],[800,300],[800,500],], np.int32)
        processed_img = self.roi(processed_img, [vertices])

        lines = cv2.HoughLinesP(processed_img, 1,  np.pi/180, 180,   20,         15)

        m1 = 0
        m2 = 0
        '''
        if lines is not None:
            l1, l2, m1, m2 = draw_lanes(original_image, lines)
            cv2.line(original_image, (l1[0], l1[1]), (l1[2], l1[3]), [0,255,0], 30)
            cv2.line(original_image, (l2[0], l2[1]), (l2[2], l2[3]), [0,255,0], 30)


        if lines is not None:
            for coords in lines:
                coords = coords[0]
                
                cv2.line(processed_img, (coords[0], coords[1]), (coords[2], coords[3]), [255,0,0], 3)     
        '''
        return processed_img, original_image, m1, m2, theta





    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        '''
        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 1.0
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.edge = cv2.Canny(blur, 60, 70)
        '''

    def detect_lines(self):
        li = np.array([[[0, 70], [0, 50], [100, 0], [540, 0], [640, 50], [640, 70]]], np.int32)
        
        new_screen, original_image, m1, m2, theta = self.process_img(self.cam_img)
        

        img = new_screen[:]

        roi_var = img[270:350, :]
        roi_var = self.roi(roi_var, li)

        self.cam_img = cv2.rectangle(roi_var, (0, self.roi_vertical_pos),
                            (self.image_width - 1, self.roi_vertical_pos + self.scan_height),
                            (255, 0, 0), 3)

        lbound = np.array([0, 0, self.value_threshold], dtype=np.uint8)
        lbound_hsv = np.array([127], dtype=np.uint8)
        ubound = np.array([131, 255, 255], dtype=np.uint8)
        ubound_hsv = np.array([255], dtype=np.uint8)
        bin = cv2.inRange(roi_var, lbound_hsv, ubound_hsv)
        view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)
        left, right = -1, -1

        for l in range(self.area_width,self.lmid):
            area = bin[self.row_begin:self.row_end, l - self.area_width:l]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                left = l
                break

        for r in range(self.image_width - self.area_width, self.rmid, -1):
            area = bin[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                right = r
                break
        roi_var = cv2.cvtColor(roi_var, cv2.COLOR_GRAY2BGR)
        if left != -1:
            lsquare = cv2.rectangle(view,
                                    (left - self.area_width, self.row_begin),
                                    (left, self.row_end),
                                    (0, 255, 0), 3)
        else:
            pass

        if right != -1:
            rsquare = cv2.rectangle(view,
                                    (right, self.row_begin),
                                    (right + self.area_width, self.row_end),
                                    (0, 255, 0), 3)
        else:
            pass
        '''
        if cv2.waitKey(1) & 0xFF == 27:
            quit()    
        cv2.imshow("view", view)
        cv2.imshow('cam_img', self.cam_img)
        '''
        # Return positions of left and right lines detected.
        return left, right, theta

    def show_images(self, left, right):
        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().
        Pass

