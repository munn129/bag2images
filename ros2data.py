#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from novatel_oem7_msgs.msg import INSPVA
from cv_bridge import CvBridge

class Ros2data():
    def __init__(self) -> None:
        self.latitude = 0
        self.longitude = 0
        self.north_vel = 0
        self.east_vel = 0
        self.image = None
        self.bridge = CvBridge()
        rospy.init_node('test', anonymous=True)
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.inspva_callback)
        rospy.Subscriber('/gmsl_camera/dev/video0/compressed', CompressedImage, self.image_callback)

    def inspva_callback(self, data) -> None:
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.north_vel = data.north_velocity
        self.east_vel = data.east_velocity

    def image_callback(self, data) -> None:
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            print(e)
            return

    def get_gps(self) -> tuple:
        return self.latitude, self.longitude
    
    def get_vel(self) -> tuple:
        return self.north_vel, self.east_vel
    
    def image_show(self) -> None:
        if self.image is not None:
            cv2.putText(self.image, 'lat : ' + str(self.latitude), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(self.image, 'lon : ' + str(self.longitude), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.imshow('image', self.image)
            cv2.resizeWindow('image', 800, 600)
            cv2.waitKey(1)
        else:
            print('\r image object is None', end='')

def main():
    ros2data = Ros2data()

    # rate = rospy.Rate(1)  # Adjust the rate as needed

    try:
        while not rospy.is_shutdown():
            if int(ros2data.get_vel()[0]) > 1 or int(ros2data.get_vel()[1]) > 1:
                print(f'\rRat: {ros2data.get_gps()[0]}, long: {ros2data.get_gps()[1]}' , end = '')
            else:
                print(f'\rvehicle is not moving...', end='')
            ros2data.image_show()
            # rate.sleep()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
