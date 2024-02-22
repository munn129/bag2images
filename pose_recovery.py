import numpy as np
import cv2

from math import sqrt

image1 = cv2.imread('./000001.png', cv2.IMREAD_GRAYSCALE)
image2 = cv2.imread('./000005.png', cv2.IMREAD_GRAYSCALE)

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

print(f'de_tran: {de_tran}, re_tran: {re_tran}')

query_gps = 37.391707531439515, 126.6445309365309
data_gps = 37.39197085599082, 126.64394834336419

scale = sqrt((query_gps[0]-data_gps[0])**2 + (query_gps[1]-data_gps[1])**2)

estimate = data_gps[0] + (scale * re_tran[0]), data_gps[1] + (scale * re_tran[1])
estimate2 = data_gps[0] + (scale * de_tran[0]), data_gps[1] + (scale * de_tran[1])


print(estimate)
print(estimate2)