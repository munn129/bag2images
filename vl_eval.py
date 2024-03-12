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
with open(result_file_path, 'r') as file:
    for line in file:
        line = line.split(', ')
        if line[0][0] == '#': continue
        
        if iter <= 5:
            retrieved_list.append(line[1][24:])
        else: pass

        if iter == 5:
            result[line[0][24:]] = retrieved_list
            retrieved_list = []

        if iter == 101: iter = 0

        iter += 1

# pprint.pprint(result)
print(result['1114/000002.png'][0][-9:-5])