from gps2meter import gps_to_meter
from dataset_distance_eval import gps_reader
from image_retrieval_eval import result_sorting, histogram_cal
from pose_recovery import image_rt_calculator, scale_calculator

from tqdm import tqdm

def estimation_error_calculator(errors, result_list):
    top1 = []
    top2 = []
    top3 = []
    top4 = []
    top5 = []

    for i in range(len(errors)):
        if i % 5 == 0:
            top1.append(errors[i])
        elif i % 5 == 1:
            top2.append(errors[i])
        elif i % 5 == 2:
            top3.append(errors[i])
        elif i % 5 == 3:
            top4.append(errors[i])
        elif i % 5 == 4:
            top5.append(errors[i])

    result_list.append(top1)
    result_list.append(top2)
    result_list.append(top3)
    result_list.append(top4)
    result_list.append(top5)

def main():
    
    dataset = './data/1024_10m'
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
    # result = {query image : [retrieved top1, top2, ... , top5], ...}
    result = {}
    retrieved_list = []
    result_sorting(result_file_path, result, retrieved_list)

    error = []
    result_list = []
    root_dir = './data'
    # for key, value in result.items():
    #     rt_list = []
    #     query_image = key
    #     query_gps = query_gps_list[int(key[-8:-4]) - 2]
    #     for i in value:
    #         retreived_image = i
    #         retrieved_gps = db_gps_list[int(i[-8:-4]) - 2]
    #         image_rt_calculator(f'{root_dir}/{query_image}', f'{root_dir}/{retreived_image}', rt_list)
    #         scale = scale_calculator(query_gps[0], query_gps[1], retrieved_gps[0], retrieved_gps[1])
    #         estimate = retrieved_gps[0] + (scale * rt_list[4][0]), retrieved_gps[1] + (scale * rt_list[4][1])
    #         # estimates.append(estimate)

    iter = 0
    for key, value in tqdm(result.items()):
        rt_list = []
        query_image = key
        query_gps = query_gps_list[int(key[-8:-4]) - 2]
        for i in value:
            retreived_image = i
            retrieved_gps = db_gps_list[int(i[-8:-4]) - 2]
            image_rt_calculator(f'{root_dir}/{query_image}', f'{root_dir}/{retreived_image}', rt_list)
            scale = scale_calculator(query_gps[0], query_gps[1], retrieved_gps[0], retrieved_gps[1])
            estimate = retrieved_gps[0] + (scale * rt_list[-1][0]), retrieved_gps[1] + (scale * rt_list[-1][1])
            error.append(gps_to_meter(query_gps[0], query_gps[1], estimate[0], estimate[1]))
        iter += 1
        if iter == 600: break

    estimation_error_calculator(error, result_list)
    histogram_cal(result_list)

if __name__ == '__main__':
    main()