#!/usr/bin/env python3
import setup_path 
import airsim
import sys
import math
import time
import argparse
import pprint
import numpy
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2 
# Makes the drone fly and get Lidar data
class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
  

    def execute(self,ptspub):
        # print("arming the drone...")
        # self.client.armDisarm(True)
        # state = self.client.getMultirotorState()
        # s = pprint.pformat(state)
        # #print("state: %s" % s)
        # airsim.wait_key('Press any key to takeoff')
        # self.client.takeoffAsync().join()
        # state = self.client.getMultirotorState()
        # #print("state: %s" % pprint.pformat(state))
        # airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
        # self.client.moveToPositionAsync(-10, 10, -10, 5).join()
        # self.client.hoverAsync().join()

        # airsim.wait_key('Press any key to get Lidar readings')
        while not rospy.is_shutdown():
            rospy.sleep(0.04)
            self.client.moveByVelocityAsync(0, 0, -2,2).join()  
            try:
                lidarData = self.client.getLidarData()
            except Exception as e:
                continue;
            if(len(lidarData.point_cloud)<30):
                continue
            points = self.parse_lidarData(lidarData)
            print("time_stamp: {} number_of_points: {}" .format(lidarData.time_stamp, len(points)))
            print("lidar position: {}" .format (pprint.pformat(lidarData.pose.position)))
            print("lidar orientation: {}" .format (pprint.pformat(lidarData.pose.orientation)))
            msg =PointCloud2()
            msg=point_cloud2.create_cloud_xyz32(msg.header,points)
            msg.header.frame_id='map'
            ptspub.publish(msg)
             

    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

    def write_lidarData_to_disk(self, points):
        # TODO
        print("not yet implemented")

    def stop(self):
        airsim.wait_key('Press any key to reset to original state')
        self.client.armDisarm(False)
        self.client.reset()
        self.client.enableApiControl(False)
        print("Done!\n")

    def pubpose(self):
        client.moveByVelocityAsync(0, 0, -5,3)


if __name__ == "__main__":
    rospy.init_node('air_sim_node')
    ptspub = rospy.Publisher("/clouds",PointCloud2,queue_size=10)
    lidarTest = LidarTest()
    lidarTest.execute(ptspub)
    lidarTest.stop()
    # rospy.spin()
