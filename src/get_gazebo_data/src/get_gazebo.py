#!/usr/bin/env python
import numpy as np

import pandas as pd
import os
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Pose

from geometry_msgs.msg import Twist

from gazebo_msgs.msg import ModelStates
import PyKDL

dataSheet = pd.DataFrame(columns=['x','y','z','r','p','y'])
def quat_to_angle(quat):
  rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
  return rot.GetRPY()
counter = 0
def callback(data):
    global counter    
    counter+=1
    if(counter%25!=0):
	return
    for i,x in enumerate(data.name):
        if x =='iris_demo':
            # print(x)
            pose=data.pose[i]
            quat=pose.orientation
            pos=pose.position
            time=rospy.get_time()
            # print(quat)
            while time==0:
                time=rospy.get_time()
            r,p,y=quat_to_angle(quat)
            dataSheet.loc[time]=[pos.x,pos.y,pos.z,r,p,y]
            print time
            # # print([pos.x,pos.y,pos.z,quat.x,quat.y,quat.z.quat.w])
            # print(quat.x,quat.y,quat.z,quat.w)
            break


rospy.init_node('get_gazebo',anonymous=True)


rospy.Subscriber('/gazebo/model_states',ModelStates,callback)

rospy.spin()

# print dataSheet
path = "/home/az/rcc/AALOG"
if not os.path.exists(path):
    os.makedirs(path)
dataSheet.to_csv(path+'/time_'+str(rospy.get_time())+'_data.csv')

print 'Data_recorded'
