import os

dataset = '1114'

output_path = f'./data/{dataset}.txt'

file_names = [i for i in range(2,3950)]

with open(output_path, 'w') as file:
    for file_name in file_names:
        file.write(f"{dataset}/{file_name:06d}.png\n")

print("complete")