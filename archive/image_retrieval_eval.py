from gps2meter import gps_to_meter
from dataset_distance_eval import gps_reader

# for debugging
import pprint

# histogram calculator
def histogram_cal(result_list):
    histogram_1 = 0
    histogram_2_5 = 0
    histogram_5 = 0
    histogram_7_5 = 0
    histogram_10 = 0
    histogram_15 = 0
    histogram_20 = 0
    histogram_30 = 0
    histogram_40 = 0
    histogram_50 = 0
    histogram_99 = 0

    for i in result_list:
        if i < 1: histogram_1 += 1
        elif i <= 1 and i < 2.5: histogram_2_5 += 1
        elif i <= 2.5 and i < 5: histogram_5 += 1
        elif i <= 5 and i < 7.5: histogram_7_5 += 1
        elif i <= 7.5 and i < 10: histogram_10 += 1
        elif i <= 10 and i < 15: histogram_15 += 1
        elif i <= 15 and i < 20: histogram_20 += 1
        elif i <= 20 and i < 30: histogram_30 += 1
        elif i <= 30 and i < 40: histogram_40 += 1
        elif i <= 40 and i < 50: histogram_50 += 1
        else: histogram_99 += 1

    print(f'error < 1 : {histogram_1}')
    print(f'1 <= error < 2.5 : {histogram_2_5}')
    print(f'2.5 <= error < 5 : {histogram_5}')
    print(f'5 <= error < 7.5: {histogram_7_5}')
    print(f'7.5 <= error < 10 : {histogram_10}')
    print(f'10 <= error < 15 : {histogram_15}')
    print(f'15 <= error < 20 : {histogram_20}')
    print(f'20 <= error < 30 : {histogram_30}')
    print(f'30 <= error < 40 : {histogram_40}')
    print(f'40 <= error < 50 : {histogram_50}')
    print(f'50 <= error : {histogram_99}')

def result_sorting(result_file_path ,result, retrieved_list):
    iter = 1
    initial_query = '1114/000002.png'
    with open(result_file_path, 'r') as file:
        for line in file:
            line = line.split(', ')
            if line[0][0] == '#': continue

            if initial_query != line[0][24:]:
                initial_query = line[0][24:]
                iter = 1

            if (initial_query == line[0][24:]) and (iter < 6):
                retrieved_list.append(line[1][24:-1])

            if iter == 5:
                result[line[0][24:]] = retrieved_list
                retrieved_list = []

            iter += 1

def error_calculator(result, query_gps_list, db_gps_list, distances, result_list):
    for key, value in result.items():
        query_gps = query_gps_list[int(key[-8:-4]) - 2]
        for i in value:
            retrieved_gps = db_gps_list[int(i[-8:-4]) - 2]
            distances.append(gps_to_meter(query_gps[0], query_gps[1], retrieved_gps[0], retrieved_gps[1]))

    top1 = []
    top2 = []
    top3 = []
    top4 = []
    top5 = []

    for i in range(len(distances)):
        if i % 5 == 0:
            top1.append(distances[i])
        elif i % 5 == 1:
            top2.append(distances[i])
        elif i % 5 == 2:
            top3.append(distances[i])
        elif i % 5 == 3:
            top4.append(distances[i])
        elif i % 5 == 4:
            top5.append(distances[i])

    result_list.append(top1)
    result_list.append(top2)
    result_list.append(top3)
    result_list.append(top4)
    result_list.append(top5)

def main():
    dataset = '1024_10m'
    result_file_path = f'./data/{dataset}/PatchNetVLAD_predictions.txt'
    gps_file_path = f'./data/{dataset}/gps.txt'
    query_gps_file_path = f'./data/1114/gps.txt'

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

    # pprint.pprint(result)
    # print(result['1114/000002.png'][0][-9:-5])

    # evaluation
    distances = []
    result_list = []
    error_calculator(result, query_gps_list, db_gps_list, distances, result_list)

    for idx, val in enumerate(result_list):
        print(f'top{idx + 1} statistics')
        print(f'min  : {min(val)}')
        print(f'max  : {max(val)}')
        print(f'mean : {sum(val)/len(val)}')

    for idx, val in enumerate(result_list):
        print(f'top{idx + 1} histogram')
        histogram_cal(val)

if __name__ == '__main__':
    main()