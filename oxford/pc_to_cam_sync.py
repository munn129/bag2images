import numpy as np

def main():
    file_path = '2015-11-10-10-32-52/pc_timestamps.npy'

    data = np.load(file_path)
    
    print(data)

if __name__ == '__main__':
    main()