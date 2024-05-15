# oxford dataset으로 image retrieval을 평가하기 위함

# Todo
# camera_timestamps.npy 중에 pc_timestamps.npy에 있는 값과 가장 가까운 것들만 추려내기
# (lidar와 image의 간격 맞추기) -> 파일로 저장
# 저 목록을 기준으로, image 리스트와 gps 리스트 생성
# 그리고 그 목록을 기준으로 새로운 데이터셋 만들기

import pandas as pd

def main():
    
    file_path = '2015-11-10-10-32-52/gps.csv'
    df = pd.read_csv(file_path, usecols=['latitude', 'longitude'])

    print(df)

if __name__ == '__main__':
    main()