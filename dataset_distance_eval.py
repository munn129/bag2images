from gps2meter import gps_to_meter

file_path = './data/1114/gps.txt'

gps_list = []

def gps_reader(filepath, save_list):
    with open(filepath, 'r') as file:
        for line in file:
            line = line.split()
            lat = float(line[1])
            lon = float(line[2])
            save_list.append((lat,lon))
            lat = 0
            lon = 0

def main():
    gps_reader(file_path, gps_list)

    distance_sum = 0

    for i in range(len(gps_list) - 1):
        distance = gps_to_meter(gps_list[i][0], gps_list[i][1], gps_list[i+1][0], gps_list[i+1][1])
        distance_sum += distance
        print(distance)

    print(f'avr distance is {distance_sum/(len(gps_list) - 1)}')

if __name__ == '__main__':
    main()