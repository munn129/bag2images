import numpy as np
import csv

def npy_to_csv(npy_file, csv_file):
    # .npy 파일 읽기
    data = np.load(npy_file)

    # CSV 파일로 데이터 쓰기
    with open(csv_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)
'''
def time_to_csv(npy_file, csv_file):

    data = np.load(npy_file)

    file = open(npy_file, 'w')
    out = csv.writer(file)
    out.writerows(map(lambda x : [x], data))
'''

# .npy 파일 경로와 변환할 CSV 파일 경로 설정
npy_file_path = 'oxford/2015-11-10-10-32-52/pc_poses.npy'
csv_file_path = 'pc_timestamps.csv'

# npy_to_csv 함수 호출
npy_to_csv(npy_file_path, csv_file_path)
