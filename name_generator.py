import os

output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output.txt')

file_names = [i for i in range(0,801)]

with open(output_path, 'w') as file:
    for file_name in file_names:
        file.write(f"image_2/{file_name:06d}.png\n")

print("complete")

