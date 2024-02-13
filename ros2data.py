#!/usr/bin/env python3

import rospy
from novatel_oem7_msgs.msg import INSPVA

class Ros2data():
    def __init__(self) -> None:
        self.latitude = 0
        self.longitude = 0
        rospy.init_node('test', anonymous=True)
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.inspva_callback)

    def inspva_callback(self, data) -> None:
        self.latitude = data.latitude
        self.longitude = data.longitude

    def get_gps(self) -> tuple:
        return self.latitude, self.longitude

if __name__ == '__main__':

    ros2data = Ros2data()

    try:
        while not rospy.is_shutdown():
            print(f'lat: {ros2data.get_gps()[0]}, long: {ros2data.get_gps()[1]}')
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

    rospy.spin()