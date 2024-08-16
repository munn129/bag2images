import os

png_files = [f for f in os.listdir() if f.endswith('.png')]

sorted_png_files = sorted(png_files, key = lambda x: int(x.split('.')[0]))

for idx, old_name in enumerate(sorted_png_files, start = 1):
    new_name = f'{idx:05d}.png'
    os.rename(old_name, new_name)
    print(f'renamed: {old_name} -> {new_name}')

