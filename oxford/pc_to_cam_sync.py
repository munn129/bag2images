# Todo
# camera_timestamps.npy 중에 pc_timestamps.npy에 있는 값과 가장 가까운 것들만 추려내기
# (lidar와 image의 간격 맞추기) -> 파일로 저장
# 저 목록을 기준으로, image 리스트와 gps 리스트 생성
# 그리고 그 목록을 기준으로 새로운 데이터셋 만들기

import numpy as np
import cv2
import pandas as pd
from tqdm import tqdm

import csv
import os

def main():

    root_path = '2015-11-10-10-32-52'

    pc_timestamps_npy = os.path.join(root_path, 'pc_timestamps.npy')
    camera_timestamps_npy = os.path.join(root_path, 'camera_timestamps.npy')
    gps_file_path = os.path.join(root_path, 'gps.csv')
    image_root_path = os.path.join(root_path, 'stereo', 'centre')
    
    output_path = 'oxford_151110'

    pc_timestamps = np.load(pc_timestamps_npy)
    camera_timestamps = np.load(camera_timestamps_npy)
    gps_pd = pd.read_csv(gps_file_path, usecols=['timestamp', 'latitude', 'longitude'])
    synced_caemra_timestamps = []
    
    # gps file 생성
    gps_output = os.path.join(output_path, 'gps.txt')
    with open(gps_output, 'w') as file:
        file.write('# image latitude longitude\n')

    # 비슷한 시간을 찾아서 매칭해야 할 줄 알았는데, 이렇게해도 됨
    for pc_time in tqdm(pc_timestamps):
        for cam_time in camera_timestamps:
            if pc_time == cam_time:
                synced_caemra_timestamps.append(cam_time)

    # imread and imwrite
    for cam_time in tqdm(synced_caemra_timestamps):

        # 이미지 읽기
        img_name = str(cam_time) + '.jpg'
        img_path = os.path.join(image_root_path, img_name)
        image = cv2.imread(img_path)
        
        # 이미지 쓰기
        img_output_path = os.path.join(output_path, img_name)
        cv2.imwrite(img_output_path, image)


        '''
        ERROR 상황
        
        라이다랑 카메라는 타임 싱크가 맞춰져 있어서,
        라이다 시간이랑 카메라 시간이랑 동일한 데이터를 선택하면 끝

        근데 gps가 안맞춰져 있어서,
        결국 가장 가까운 값을 찾아야 함
        '''

        # camera time에 해당하는 gps 읽기
        gps = gps_pd[gps_pd['timestamp'] == int(cam_time)]
        latitude = gps['latitude'].values[0]
        longitude = gps['longitude'].values[0]

        # gps write(Kapture format)
        with open(gps_output, 'a') as file:
            file.write(f'{img_output_path} {latitude} {longitude}\n')
        

if __name__ == '__main__':
    main()