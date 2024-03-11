#!/usr/bin/env python3

from math import sqrt

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from novatel_oem7_msgs.msg import INSPVA
from cv_bridge import CvBridge

from gps2meter import gps_to_meter

class Ros2data():
    def __init__(self) -> None:
        self.latitude = 0
        self.longitude = 0
        self.height = 0
        self.north_vel = 0
        self.east_vel = 0
        self.image = None
        self.bridge = CvBridge()
        self.time = 0
        self.seq = 0
        self.image_flag = False
        self.gps_flag = False
        rospy.init_node('test', anonymous=True)
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.inspva_callback)
        rospy.Subscriber('/gmsl_camera/dev/video0/compressed', CompressedImage, self.image_callback)

    def inspva_callback(self, data) -> None:

        if self.gps_flag:
            self.latitude = data.latitude
            self.longitude = data.longitude
            self.height = data.height
            self.north_vel = data.north_velocity
            self.east_vel = data.east_velocity
            self.time = data.header.stamp.secs
            self.seq = data.header.seq
            self.gps_flag = False

    def image_callback(self, data) -> None:
        try:
            if self.image_flag:
                self.image = self.bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
                self.image_flag = False
                
        except Exception as e:
            print(e)
            return
        
    def get_time(self) -> int:
        return self.time

    def get_gps(self) -> tuple:
        return self.latitude, self.longitude, self.height
    
    def get_vel(self) -> float:
        return sqrt(self.north_vel**2 + self.east_vel**2)
    
    def get_image(self) -> list:
        return self.image

    def get_seq(self) -> int:
        return self.seq
    
    def image_show(self) -> None:
        if self.image is not None:
            image_copy = self.image[:]
            cv2.putText(image_copy, 'lat : ' + str(self.latitude), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(image_copy, 'lon : ' + str(self.longitude), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(image_copy, 'vel : ' + str(self.get_vel()), (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.imshow('image', image_copy)
            cv2.resizeWindow('image', 800, 500)
            cv2.waitKey(1)
        else:
            print('\r image object is None', end='')

def data_writer(gps, image, idx) -> None:
    
    image_name = idx
    dataset = '1024_10m'

    if image is not None:
        with open(f'./data/{dataset}/gps.txt', 'a') as file:
            file.write(f'{dataset}/{image_name:06d}.png {gps[0]} {gps[1]}\n')

        cv2.imwrite(f'./data/{dataset}/{image_name:06d}.png', image)

def only_gps_writer(gps, idx) -> None:
    dataset = 'test'
    with open(f'./data/{dataset}/gps.txt', 'a') as file:
        file.write(f'{idx} {gps[0]} {gps[1]}\n')

def main():
    ros2data = Ros2data()
    sleep_time = 0.001
    init_gps = 0,0
    distance = 0
    idx = 1

    try:
        while not rospy.is_shutdown():
            if init_gps == 0:
                init_gps[0] = ros2data.get_gps()[0]
                init_gps[1] = ros2data.get_gps()[1]

            ros2data.image_flag = True
            ros2data.gps_flag = True

            distance = gps_to_meter(init_gps[0], init_gps[1], ros2data.get_gps()[0], ros2data.get_gps()[1])

            # if (ros2data.gps_flag):
            #     only_gps_writer(ros2data.get_gps(), idx)
            #     idx += 1
            #     rospy.sleep(sleep_time)

            if (distance > 9.8 and ros2data.image_flag and ros2data.image_flag):
                print(f'\rRat: {ros2data.get_gps()[0]}, long: {ros2data.get_gps()[1]}, idx: {idx}' , end = '          ')
                # ros2data.image_show()
                data_writer(ros2data.get_gps(), ros2data.get_image(), idx)
                rospy.sleep(sleep_time)
                idx += 1
                init_gps = ros2data.get_gps()[0], ros2data.get_gps()[1]
            else:
                print('\r ====================', end='                                            ')

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
