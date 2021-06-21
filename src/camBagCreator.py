import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from PIL import ImageFile
import time, sys, os
import argparse
import cv2
import numpy as np
import csv
import os

# path = os.path.abspath(os.getcwd())
# os.chdir(path)

def getImageFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    image_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    image_files.append( os.path.join( path, f ) )
                    timestamps.append(os.path.splitext(f)[0]) 
    #sort by timestamp
    sort_list = sorted(zip(timestamps, image_files))
    image_files = [file[1] for file in sort_list]
    return image_files


def loadImageToRosMsg(filename,erasedSamplesCam,adjusted_camtime,i):
    '''
    for each image, ros syntax timestamps and raw image content is written to bags
    '''
    image_np = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    
    timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
    timestamp = rospy.Time( secs=int(timestamp_nsecs[0:-9])-11644473600, nsecs=int(timestamp_nsecs[-9:]) )

    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.height = image_np.shape[0]
    rosimage.width = image_np.shape[1]
    rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
    rosimage.encoding = "mono8"
    rosimage.data = image_np.tostring()
    
    return rosimage, timestamp

    
def make(erasedSamplesCam, adjusted_camtime):
    '''
    creates rosbag from hl images in a for loop for each hl image 
    excess samples are erased via the usage of coarse alignment
    main source code was ripped and modded from kalibr utility scripts.
    '''
    #create the bag
    try:
        bag = rosbag.Bag("hl_cam.bag", 'w')
        #write images
        camdir = "PV"
        i = 0
        image_files = getImageFilesFromDir(camdir)
        for image_filename in image_files:
            if i>= erasedSamplesCam:
                image_msg, timestamp = loadImageToRosMsg(image_filename,erasedSamplesCam,adjusted_camtime,i)
                #print("i is %f" %i)
                bag.write("/cam0/image_raw", image_msg, timestamp)
            i = i+1
            if i == (len(adjusted_camtime)):
            #if i ==len(adjusted_camtime):
                break
    finally:
        bag.close()
