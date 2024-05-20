# oxford dataset으로 image retrieval을 평가하기 위함

# Todo
# camera_timestamps.npy 중에 pc_timestamps.npy에 있는 값과 가장 가까운 것들만 추려내기
# (lidar와 image의 간격 맞추기) -> 파일로 저장
# 저 목록을 기준으로, image 리스트와 gps 리스트 생성
# 그리고 그 목록을 기준으로 새로운 데이터셋 만들기

import pandas as pd
from tqdm import tqdm

def main():
    
    root = '151113gps.txt'

    time_file = 'oxford_151113_filename_list.txt'
    gps_file = '151113gps/gps/gps.csv'
    
    # 이미지가 촬영된 시간
    camera_timestamp = []
    camera_namelist = []

    with open(time_file, 'r') as file:
        for line in file:
            # oxford_151110/1447151628069092.jpg
            camera_namelist.append(line)
            camera_timestamp.append(line.split('/')[-1].split('.')[0])
    
    gps = pd.read_csv(gps_file, usecols=['timestamp', 'latitude', 'longitude'])

    # print(gps[gps['timestamp'] == int(1447153502989486)]['latitude'].values[0])
    # print(gps['timestamp'].values)
    
    # gps['timestamp'].values -> gps가 기록된 시간
    
    # print(str(camera_timestamp[1])[:11])

    dataset_gps = []
    idx = 0
    inter_cnt = 0
    total_time = 0
    for cam_time in tqdm(camera_timestamp):
        for i in range(idx, len(gps['timestamp'].values) - 1):
            pres = int(gps['timestamp'].values[i])
            next = int(gps['timestamp'].values[i + 1])
            lat = float(gps[gps['timestamp'] == int(pres)]['latitude'].values[0])
            lon = float(gps[gps['timestamp'] == int(pres)]['longitude'].values[0])
            if cam_time == pres:
                idx += 1
                dataset_gps.append((lat, lon))
                break
            if pres < int(cam_time) and next > int(cam_time):
                idx += 1
                lat_next = float(gps[gps['timestamp'] == int(pres)]['latitude'].values[0])
                lon_next = float(gps[gps['timestamp'] == int(pres)]['longitude'].values[0])
                dataset_gps.append(((lat + lat_next)/2 , (lon + lon_next)/2))
                total_time += abs(int(cam_time) - pres)
                break

    for i in dataset_gps:
        with open(f'{root}_result.txt', 'a') as file:
            file.write(f'{i} \n')
    
    print(f'average time gap : {total_time/idx}')
                
    # print(len(dataset_gps))

    with open(root, 'w') as file:
        file.write('# image latitude longitude\n')

    # gps write(Kapture format)
    for i in range(len(camera_namelist)):
        with open(root, 'a') as file:
            file.write(f'{camera_namelist[i][:-1]} {dataset_gps[i][0]} {dataset_gps[i][1]}\n')

if __name__ == '__main__':
    main()