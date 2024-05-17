a = [1,2,3,4,5]
b = ['a','b','c','d','e']

idx = 0
for j in a:
    idx += 1
    for i in range(idx, len(b)):
        print(i, j)