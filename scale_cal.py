from math import sqrt

'''
a,c -> query
b,d -> datasets
'''
a = 37.39258759506305
b = 37.39309396083113
c = 126.64301375035947
d = 126.64211532287065

scale = sqrt((a-b)**2 + (c-d)**2)

trans_x = -0.80275555
trans_y = -0.80275555

estimate = b + (scale * trans_x), d + (scale * trans_y)

print(estimate)