#!/usr/bin/env python3

import numpy as np
from math import*

def calc_dist(point1,point2):
    return  sqrt(pow(point1[0]-point2[0],2)
                +pow(point1[1]-point2[1],2)
                +pow(point1[2]-point2[2],2))
def waypoints_corride(waypoints):
    
    waypoints_copy = waypoints
    waypoints_rows = waypoints.shape[0]
    print(waypoints_rows)
    waypoints_new = np.array([])
    for i in range(waypoints_rows-1):
        if(calc_dist(waypoints_copy[i],waypoints_copy[i+1]>3)):
            waypoints_new=np.append(waypoints_new,np.linspace(waypoints_copy[i],waypoints_copy[i+1],3,endpoint=False))
            print(waypoints_new)
        else:
            waypoints_new=np.append(waypoints_new,waypoints_copy[i])
            print('no')

    return np.reshape(waypoints_new,(-1,4))


waypoints=np.array([[1,2,3,4],[5,6,7,8]])

print (waypoints_corride(waypoints))