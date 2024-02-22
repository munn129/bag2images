import os

output_path = './rain_3rd_lane_list.txt'

file_names = [i for i in range(1,35)]

with open(output_path, 'w') as file:
    for file_name in file_names:
        file.write(f"rain_3rd_lane/{file_name:06d}.png\n")

print("complete")

