import rosbag
import rospy
from sensor_msgs.msg import Imu
import time, sys, os
import argparse
import cv2
import numpy as np
import csv
import os



def getImuFiles(dir):
    '''obtain hl formatted text'''
    imu_files = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for file in files:
                if file == 'hlmatrix.txt':
                    imu_files.append( os.path.join( path, file ) )
    
    return imu_files

def createImuMessge(hltimestamp, imuVars):
    
    tempTime= hltimestamp
    tempTime = str(tempTime)    
    timestamp = rospy.Time( int(tempTime[0:10]), int(tempTime[11:]))
    
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.linear_acceleration.x = float(imuVars[0])
    rosimu.linear_acceleration.y = float(imuVars[1])
    rosimu.linear_acceleration.z = float(imuVars[2])
    rosimu.angular_velocity.x    = float(imuVars[3])
    rosimu.angular_velocity.y    = float(imuVars[4])
    rosimu.angular_velocity.z    = float(imuVars[5])
    
    return rosimu, timestamp
    
#create the bag
def make():
    '''
    creates rosbags with post-processed imu data coming from the framework
    '''
    try:
        bag = rosbag.Bag("hl_imu.bag", 'w')
        cwd = os.getcwd()
        #write imu data
        imufiles = getImuFiles(cwd)
        for imufile in imufiles:
            topic = os.path.splitext(os.path.basename(imufile))[0]
            with open(imufile, 'rt') as csvfile:
                reader = csv.reader(csvfile, delimiter=' ')
                for row in reader:
                    imumsg, timestamp = createImuMessge(row[6], row[0:6])
                    bag.write("/imu1", imumsg, timestamp)
    finally:
        bag.close()
