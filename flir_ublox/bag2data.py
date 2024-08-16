#!/usr/bin/env python3

## NOT WORKING ##

import os

import rospy
import message_filters
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ublox_msgs.msg import NavPVT

image_cnt = 0
gps_cnt = 0

image_dir = './images'
gps_dir = 'gps.txt'

if not os.path.exists(image_dir):
    os.makedirs(image_dir)

bridge = CvBridge()

def callback(image, pvt_msg):
    global image_cnt, gps_cnt
    
    rospy.loginfo(f'Image timestamp : {image.header.stamp} \n GPS timestamp : {pvt_msg.header.stamp}')

    flir = bridge.imgmsg_to_cv2(image, 'bgr8')
    cv2.imwrite(os.path.join(image_dir, f'{image_cnt:06}.png'), flir)

    lat = int(pvt_msg.lat) / 10000000
    lon = int(pvt_msg.lon) / 10000000
    heading = int(pvt_msg.heading) / 100000

    with open(f'{gps_dir}', 'a') as file:
        file.write(f'{image_dir}/{image_cnt:06}.png {lat} {lon} {heading}')

    image_cnt += 1
    gps_cnt += 1

    rospy.loginfo(f'{image_cnt}-th Image, {gps_cnt}-th GPS saved...')

def main():
    try:
        while not rospy.is_shutdown():
            rospy.init_node('syncronized_data', anonymous=True)

            image_sub = message_filters.Subscriber('/camera/image_raw', Image)
            gps_sub = message_filters.Subscriber('/ublox/navpvt', NavPVT)

            time_syncronize = message_filters.ApproximateTimeSynchronizer([image_sub, gps_sub], queue_size = 10, slop = 0.1)
            time_syncronize.registerCallback(callback)
    
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()