from gps2meter import gps_to_meter
from dataset_distance_eval import gps_reader

# for debugging
import pprint

dataset = '1024_1m'
result_file_path = f'./data/{dataset}/PatchNetVLAD_predictions.txt'
gps_file_path = f'./data/{dataset}/gps.txt'
query_gps_file_path = f'./data/1114/gps.txt'

# gps list index 0 -> 000002.png, 1 -> 000003.png, ...
# read query gps list
query_gps_list = []
gps_reader(query_gps_file_path, query_gps_list)
print('query gps list completes')

# read db gps list
db_gps_list = []
gps_reader(gps_file_path, db_gps_list)
print('db gps list completes')

# read image retrieval result
iter = 1
result = {}
retrieved_list = []
initial_query = '1114/000002.png'
with open(result_file_path, 'r') as file:
    for line in file:
        line = line.split(', ')
        if line[0][0] == '#': continue

        if initial_query != line[0][24:]:
            initial_query = line[0][24:]
            iter = 1

        if initial_query == line[0][24:] and iter < 6:
            retrieved_list.append(line[1][24:])

        if iter == 5:
            result[line[0][24:]] = retrieved_list
            retrieved_list = []

        # if iter == 101: iter = 0

        iter += 1

pprint.pprint(result)
# print(result['1114/000002.png'][0][-9:-5])

# evaluation
distances = []

for key, value in result.items():
    query_gps = query_gps_list[int(key[-8:-4]) - 2]
    for i in value:
        retrieved_gps = db_gps_list[int(i[-9:-5]) - 2]
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

arr = []
arr.append(top1)
arr.append(top2)
arr.append(top3)
arr.append(top4)
arr.append(top5)

for idx, val in enumerate(arr):
    print(f'top{idx + 1} statistics')
    print(f'min  : {min(val)}')
    print(f'max  : {max(val)}')
    print(f'mean : {sum(val)/len(val)}')
