a = [1,2,3,4,5]
b = ['a','b','c','d','e']

for i in a:
    with open('result_save.txt', 'a') as file:
            file.write(f'{i}\n')