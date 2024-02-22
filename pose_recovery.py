import numpy as np
import cv2

from math import sqrt, pi, sin, cos, atan2

def gps_to_meter(lat1, long1, lat2, long2) -> float:
    R = 6378.137 # radius of the earth in KM
    lat_to_deg = lat2 * pi/180 - lat1 * pi/180
    long_to_deg = long2 * pi/180 - long1 * pi/180

    a = sin(lat_to_deg/2)**2 + cos(lat1 * pi/180) * cos(lat2 * pi/180) * sin(long_to_deg/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = R * c
    
    return d * 1000 # meter

image1 = cv2.imread('./000001.png', cv2.IMREAD_GRAYSCALE)
image2 = cv2.imread('./000011.png', cv2.IMREAD_GRAYSCALE)

sift_obj = cv2.SIFT_create()

keypoint1, descriptor1 = sift_obj.detectAndCompute(image1, None)
keypoint2, descriptor2 = sift_obj.detectAndCompute(image2, None)

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

flann = cv2.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(descriptor1, descriptor2, k = 2)

points1 = []
points2 = []

for i, (m,n) in enumerate(matches):
    if m.distance < 0.8 * n.distance:
        points2.append(keypoint2[m.trainIdx].pt)
        points1.append(keypoint1[m.queryIdx].pt)

points1 = np.int32(points1)
points2 = np.int32(points2)

E, mask = cv2.findEssentialMat(points1, points2)

de_rot1, de_rot2, de_tran = cv2.decomposeEssentialMat(E)
re_retval, re_rot, re_tran, re_mask = cv2.recoverPose(E, points1, points2)

query_gps = 37.39246703655914, 126.64313877241125
data_gps = 37.392469887350785, 126.64313416569627

scale = sqrt((query_gps[0]-data_gps[0])**2 + (query_gps[1]-data_gps[1])**2)
print(scale)

estimate = data_gps[0] + (scale * re_tran[0]), data_gps[1] + (scale * re_tran[1])
estimate2 = data_gps[0] + (scale * de_tran[0]), data_gps[1] + (scale * de_tran[1])

distance = gps_to_meter(data_gps[0], data_gps[1], estimate[0], estimate[1])
distance2 = gps_to_meter(data_gps[0], data_gps[1], estimate2[0], estimate2[1])
gt = gps_to_meter(query_gps[0], query_gps[1], data_gps[0], data_gps[1])

print(f'decompose method: {distance}, error: {distance-gt}\n recover method: {distance2}, error: {distance2 - gt}\n')