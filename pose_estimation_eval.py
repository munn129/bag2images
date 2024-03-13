from gps2meter import gps_to_meter
from dataset_distance_eval import gps_reader
from image_retrieval_eval import result_sorting
from pose_recovery import image_rt_calculator

'''
Todo list.
1. 쿼리 이미지 입력에 해당하는 image retrieval list
2. 쿼리 이미지 <-> retrieved image 사이 R|t
3. 쿼리 이미지 GPS, retrieved image GPS 사이 거리 계산(gt, scale)
4. 평가 코드
'''

def main():
    
    dataset = './data/1024_1m'
    query = './data/1114'
    result_file_path = f'{dataset}/PatchNetVLAD_predictions.txt'
    gps_file_path = f'{dataset}/gps.txt'
    query_gps_file_path = f'{query}/gps.txt'

    # gps list index 0 -> 000002.png, 1 -> 000003.png, ...
    # read query gps list
    # [(lat1, lon1), (lat2, lon2), ...]
    query_gps_list = []
    gps_reader(query_gps_file_path, query_gps_list)
    print('query gps list completes')

    # read db gps list
    # [(lat1, lon1), (lat2, lon2), ...]
    db_gps_list = []
    gps_reader(gps_file_path, db_gps_list)
    print('db gps list completes')

    # read image retrieval result
    # result = {'1114/003933.png' : ['1024_1m/002551.png', ... ], ...}
    result = {}
    retrieved_list = []
    result_sorting(result_file_path, result, retrieved_list)

if __name__ == '__main__':
    main()