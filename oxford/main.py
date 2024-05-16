# oxford dataset으로 image retrieval을 평가하기 위함

# Todo
# camera_timestamps.npy 중에 pc_timestamps.npy에 있는 값과 가장 가까운 것들만 추려내기
# (lidar와 image의 간격 맞추기) -> 파일로 저장
# 저 목록을 기준으로, image 리스트와 gps 리스트 생성
# 그리고 그 목록을 기준으로 새로운 데이터셋 만들기

import pandas as pd
from tqdm import tqdm

def main():
    
    time_file = 'oxford_151110_filename_list.txt'
    gps_file = '151110gps/gps.csv'
    
    # 이미지가 촬영된 시간
    camera_timestamp = []

    with open(time_file, 'r') as file:
        for line in file:
            # oxford_151110/1447151628069092.jpg
            camera_timestamp.append(line.split('/')[-1].split('.')[0])
    
    gps = pd.read_csv(gps_file, usecols=['timestamp', 'latitude', 'longitude'])

    # print(gps[gps['timestamp'] == int(1447153502989486)]['latitude'].values[0])
    # print(gps['timestamp'].values)
    
    # gps['timestamp'].values -> gps가 기록된 시간
    
    # print(str(camera_timestamp[1])[:11])

    dataset_timestamp = []
    for cam_time in tqdm(camera_timestamp):
        for gps_time in gps['timestamp'].values:
            if str(cam_time)[:11] == str(gps_time)[:11]:
                latitude = gps[gps['timestamp'] == int(gps_time)]['latitude'].values[0]
                longitude = gps[gps['timestamp'] == int(gps_time)]['longitude'].values[0]
                dataset_timestamp.append((latitude, longitude))
                continue
        
    print(dataset_timestamp)

if __name__ == '__main__':
    main()