from gps2meter import gps_to_meter

file_path = './data/1024_10m/gps.txt'

gps_list = []

with open(file_path, 'r') as file:
    for line in file:
        line = line.split()
        lat = float(line[1])
        lon = float(line[2])
        gps_list.append((lat, lon))
        lat = 0
        lon = 0

distance_sum = 0

for i in range(len(gps_list) - 1):
    distance = gps_to_meter(gps_list[i][0], gps_list[i][1], gps_list[i+1][0], gps_list[i+1][1])
    distance_sum += distance
    print(distance)

print(f'avr distance is {distance_sum/(len(gps_list) - 1)}')