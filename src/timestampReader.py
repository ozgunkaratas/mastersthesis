import matplotlib.pyplot as plt
import numpy as np
import math
from decimal import Decimal
import pathlib
import os, sys, time


CONST_EPOCH = 11644473600000000000 #nseconds between win epoch and unix epoch
'''
various functions to read the timestamps of the hl images, align internal hl imu and hl cam timelines and convert
from hundreds of nanosecs (windows default) to full nanosecs(epoch)
'''
def file_command(filepath):
    f = open(filepath,'r')
    name = f.name
    realname = os.path.basename(f.name)
    realname = realname.split('.')
    realname = realname[0]
    f.close()
    with open('pv_timestamps.txt','a+') as fn:    
        fn = open("pv_timestamps.txt","a+")
        fn.write(realname+"\n")
    


def createCamFile():
    '''
    converts hundreds of nsecs to nsecs in wintime.
    '''
    for path in pathlib.Path("PV").iterdir():
        if path.is_file():
            old_name = path.stem
            old_extension = path.suffix
            directory = path.parent
            new_name =old_name + "00" + old_extension
            path.rename(pathlib.Path(directory, new_name))

    for filename in os.listdir("PV"):
        filepath = os.path.join("PV",filename)
        file_command(filepath)


def initialize(h_a_time_epoch, h_g_time_epoch):
    pvStamps = np.loadtxt('pv_timestamps.txt', dtype= Decimal)
    pvStamps_secs = np.arange(len(pvStamps),dtype = np.float64)
    pvStamps_nsecs = np.arange(len(pvStamps),dtype = np.float64) 
    pvStamps_epoch = np.arange(len(pvStamps),dtype = Decimal)
    pvStamps_experiment = np.arange(len(pvStamps),dtype = Decimal)
    decs= np.arange(len(pvStamps),dtype = Decimal)
    decns= np.arange(len(pvStamps),dtype = Decimal)

    
    for i in range(len(pvStamps)):    
        pvStamps[i]       = int(pvStamps[i]) -CONST_EPOCH  #in total nanosecs
        tempStr           = str(pvStamps[i])
        pvStamps_secs[i]  = float(tempStr[0:10])                  #get only secs
        pvStamps_nsecs[i] = float(tempStr[10:20])                 #get only nsecs
        
        decs[i] =Decimal(pvStamps_secs[i])
        decns[i] = Decimal(pvStamps_nsecs[i])
        
        pvStamps_epoch[i] = decs[i] + decns[i]*Decimal(1e-9)
        tempStr = str(pvStamps_epoch[i])
        tempStr = tempStr[0:20]
        pvStamps_epoch[i] = Decimal(tempStr)
        pvStamps_experiment[i] = pvStamps_epoch[i] - pvStamps_epoch[0]        

   
    #duration in total seconds    
    pvDuration = pvStamps_experiment[-1] - pvStamps_experiment[0]
    
    #calculate accel duration
    hlDurationAcc = (h_a_time_epoch[-1]) - (h_a_time_epoch[0])

    #calculate gyro duration
    hlDurationGyr = (h_g_time_epoch[-1]) - (h_g_time_epoch[0])

    #calculate magno duration
    #               this will also be returned
    return(pvDuration, pvStamps_epoch,
           pvStamps_experiment,
           hlDurationAcc, hlDurationGyr)

def alignTimelines(pvDuration, hlDurationAcc,hlDurationGyr,
                   h_a_time_experiment,h_g_time_experiment,
                   hlGyrX, hlGyrY, hlGyrZ,
                   hlAccX, hlAccY, hlAccZ):
    #determine the last viable sample of accel
    for i in range(len(h_a_time_experiment)):
        diff = pvDuration - h_a_time_experiment[i]
        if diff < 0.001:
            hlDurationAcc = h_a_time_experiment[i]
            endSampleAcc  = i
            break
    #determine the last viable sample of gyro        
    for i in range(len(h_g_time_experiment)):
        diff = pvDuration - h_g_time_experiment[i]
        if diff < 0.001:
            hlDurationGyr = h_g_time_experiment[i]
            endSampleGyr  = i
            break   
        
    
    h_a_time_experiment_aug = np.arange(endSampleAcc,dtype = Decimal)
    h_g_time_experiment_aug = np.arange(endSampleGyr,dtype = Decimal)
    
        
    #erase buggy IMU data after PV stops
    hlAccX = hlAccX[0:len(h_a_time_experiment_aug)]
    hlAccY = hlAccY[0:len(h_a_time_experiment_aug)]
    hlAccZ = hlAccZ[0:len(h_a_time_experiment_aug)]

    hlGyrX = hlGyrX[0:len(h_g_time_experiment_aug)]
    hlGyrY = hlGyrY[0:len(h_g_time_experiment_aug)]
    hlGyrZ = hlGyrZ[0:len(h_g_time_experiment_aug)]

    #create augmented timeline that matches the camera   
    for i in range(endSampleAcc):
        h_a_time_experiment_aug[i] =  h_a_time_experiment[i] - h_a_time_experiment[0]
    for i in range(endSampleGyr):
        h_g_time_experiment_aug[i] =  h_g_time_experiment[i] - h_g_time_experiment[0]
        
    #convert Decimal obj to float
    h_a_time_experiment_aug = np.array(list(map(np.float64,h_a_time_experiment_aug)))
    h_g_time_experiment_aug = np.array(list(map(np.float64,h_g_time_experiment_aug))) 
    
    return(h_a_time_experiment_aug,h_g_time_experiment_aug,
           hlDurationAcc, hlDurationGyr,endSampleAcc, endSampleGyr,
           hlGyrX, hlGyrY, hlGyrZ,
           hlAccX, hlAccY, hlAccZ)

    
def estimateFPS(pvDuration,pvStamps_experiment,utcTime,
                h_a_time_experiment_aug, h_g_time_experiment_aug,
                endSampleAcc, endSampleGyr):
    #float FPS values
    pvFPS    = float(len(pvStamps_experiment)) / float(pvDuration)
    accFPS   = float(endSampleAcc) / float(h_a_time_experiment_aug[-1])
    gyrFPS   = float(endSampleGyr) / float(h_g_time_experiment_aug[-1])
    xsensFPS = float(len(utcTime)) / float(utcTime[-1])
    
    #avg FPS values
    avg_pvFPS = np.floor(pvFPS)
    avg_accFPS = np.floor(accFPS)
    avg_gyrFPS = np.floor(gyrFPS)
    avg_xsensFPS = np.floor(xsensFPS)
    
    return(pvFPS, accFPS, gyrFPS, xsensFPS,
           avg_pvFPS, avg_accFPS, avg_gyrFPS, avg_xsensFPS)
    