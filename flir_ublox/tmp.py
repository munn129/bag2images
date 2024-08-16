#!/usr/bin/env python3

from math import sqrt

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from ublox_msgs.msg import NavPVT
from cv_bridge import CvBridge

class Ros2data():
    def __init__(self) -> None:
        self.latitude = 0
        self.longitude = 0
        self.height = 0
        self.azimuth = 0
        self.north_vel = 0
        self.east_vel = 0
        self.image = None
        self.bridge = CvBridge()
        self.time = 0
        self.seq = 0
        self.image_flag = False
        self.gps_flag = False
        rospy.init_node('test', anonymous=True)
        rospy.Subscriber('/ublox/navpvt', NavPVT, self.inspva_callback)
        rospy.Subscriber('/camera/image_color/compressed', CompressedImage, self.image_callback)

    def inspva_callback(self, data) -> None:

        if self.gps_flag:
            self.latitude = int(data.lat) / 10000000
            self.longitude = int(data.lon) / 10000000
            
            self.azimuth = int(data.heading) / 100000

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
    
    def get_azimuth(self) -> float:
        return self.azimuth
    
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

def data_writer(gps, image, idx, azimuth, dataset_dir) -> None:
    
    image_name = idx
    dataset = dataset_dir

    if image is not None:
        with open(f'./data/{dataset}/gps.txt', 'a') as file:
            file.write(f'{dataset}/{image_name:06d}.png {gps[0]} {gps[1]} {azimuth}\n')

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

            dataset_dir = 'd'
            if (ros2data.image_flag and ros2data.image_flag):
                print(f'\rRat: {ros2data.get_gps()[0]}, long: {ros2data.get_gps()[1]}, idx: {idx}' , end = '          ')
                # ros2data.image_show()
                data_writer(ros2data.get_gps(), ros2data.get_image(), idx, ros2data.get_azimuth(), dataset_dir)
                rospy.sleep(sleep_time)
                idx += 1
                init_gps = ros2data.get_gps()[0], ros2data.get_gps()[1]
            
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
