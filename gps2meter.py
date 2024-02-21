'''
gps to meter calculation using Haversine formula
'''

from math import pi, sin, cos, atan2, sqrt

def gps_to_meter(lat1, long1, lat2, long2) -> float:
    R = 6378.137 # radius of the earth in KM
    lat_to_deg = lat2 * pi/180 - lat1 * pi/180
    long_to_deg = long2 * pi/180 - long1 * pi/180

    a = sin(lat_to_deg/2)**2 + cos(lat1 * pi/180) * cos(lat2 * pi/180) * sin(long_to_deg/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = R * c
    
    return d * 1000 # meter

distance = gps_to_meter(37, 126, 37, 126.00001)

print(distance)

'''
reference : 37, 126

-> 37.00001, 126,00001 : 1.42463[m]
-> 37.00001, 126 : 1.11319[m]
-> 37, 126.00001 : 0.88903[m]
'''