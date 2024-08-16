import numpy as np
import cv2

from gps2meter import gps_to_meter

from math import sqrt
from tqdm import tqdm
import time

def image_rt_calculator(query_image_path, dataset_image_path, rt_list):
    
    query_image = cv2.imread(query_image_path, cv2.IMREAD_GRAYSCALE)
    dataset_image = cv2.imread(dataset_image_path, cv2.IMREAD_GRAYSCALE)
    ratio = 4
    query_image = cv2.resize(query_image, dsize=(int(query_image.shape[0]/ratio), int(query_image.shape[1]/ratio)))
    dataset_image = cv2.resize(dataset_image, dsize=(int(dataset_image.shape[0]/ratio), int(dataset_image.shape[1]/ratio)))

    if (query_image is None or dataset_image is None): raise Exception("[ERROR] image == None")

    sift_obj = cv2.SIFT_create()

    # start = time.time()
    query_keypoint, query_descriptor = sift_obj.detectAndCompute(query_image, None)
    dataset_keypoint, dataset_descriptor = sift_obj.detectAndCompute(dataset_image, None)
    # print(f'detect and compute time: {time.time() - start:.4f} sec')

    # start = time.time()
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(query_descriptor, dataset_descriptor, k = 2)
    # print(f'descriptor matching time: {time.time() - start:.4f} sec')
    
    query_points = []
    dataset_points = []

    for i, (m,n) in enumerate(matches):
        if m.distance < 0.8 * n.distance:
            # if m.distance is under 0.5, error occurs
            dataset_points.append(dataset_keypoint[m.trainIdx].pt)
            query_points.append(query_keypoint[m.queryIdx].pt)

    query_points = np.int32(query_points)
    dataset_points = np.int32(dataset_points)

    E, mask = cv2.findEssentialMat(query_points, dataset_points)

    de_rot1, de_rot2, de_tran = cv2.decomposeEssentialMat(E)
    re_retval, re_rot, re_tran, re_mask = cv2.recoverPose(E, query_points, dataset_points)

    rt_list.append(de_rot1)
    rt_list.append(de_rot2)
    rt_list.append(de_tran)
    rt_list.append(re_rot)
    rt_list.append(re_tran)

def scale_calculator(q_lat, q_lon, d_lat, d_lon):
    return sqrt((q_lat-d_lat)**2 + (q_lon-d_lon)**2)

def main():
    query_image_path = './data/1114/000002.png'
    dataset_image_path = './data/1024_10m/000002.png'
    rt_list = []

    image_rt_calculator(query_image_path, dataset_image_path, rt_list)

    de_tran = rt_list[2]
    re_tran = rt_list[4]

    query_gps = 37.39209578665124, 126.64374551219875
    data_gps = 37.39210987530388, 126.64372867826643

    scale = scale_calculator(query_gps[0], query_gps[1], data_gps[0], data_gps[1])
    print(scale)

    estimate = data_gps[0] + (scale * re_tran[0]), data_gps[1] + (scale * re_tran[1])
    estimate2 = data_gps[0] + (scale * de_tran[0]), data_gps[1] + (scale * de_tran[1])

    distance = gps_to_meter(query_gps[0], query_gps[1], estimate[0], estimate[1])
    distance2 = gps_to_meter(data_gps[0], data_gps[1], estimate2[0], estimate2[1])
    gt = gps_to_meter(query_gps[0], query_gps[1], data_gps[0], data_gps[1])

    print(f'decompose method: {distance2}, error: {distance2-gt}\nrecover method: {distance}, error: {distance - gt}\nretrieved: {gt}')
    
if __name__ == '__main__':
    main()