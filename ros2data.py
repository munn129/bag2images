#!/usr/bin/env python3

import rospy
from novatel_oem7_msgs.msg import INSPVA

def inspva_callback(data):
    lat = data.latitude
    lon = data.longitude

def listener():
    rospy.init_node('inspva_listener', anonymous=True)
    rospy.Subscriber("/novatel/oem7/inspva", INSPVA, inspva_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()