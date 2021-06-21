import matplotlib.pyplot as plt
import numpy as np
import csv
from decimal import Decimal


CONST_EPOCH = 11644473600000000000 #nseconds between win epoch and unix epoch

def parse():
    '''
    parses formatted text obtained from hololens archive, aligns timelines as well.
    '''
    ## String Pattern for Accel and Gyro
    #acclstr = "header ax ay az norm delta wintime universaltime"
    #gyrostr = "header gx gy gz norm delta wintime universaltime"
    
    ##extract and create new hl imu files
    with open ('accel.txt', 'r') as accf:
        txt_readeracc = accf.readlines()
        
    with open ('gyro.txt', 'r') as gyrf:
        txt_readergyr = gyrf.readlines()
    
    
    for i in range(len(txt_readeracc)):
        txt_readeracc[i] = txt_readeracc[i].split() 
        
    for i in range(len(txt_readergyr)):
        txt_readergyr[i] = txt_readergyr[i].split()
    
    
    with open('accelnew.txt','w+',newline='') as csv_writer:
        for it in range(len(txt_readeracc)):  
            time = int(txt_readeracc[it][6])
            adjusted_time = int(( time*100 )-( CONST_EPOCH ))
            extr = [txt_readeracc[it][1],txt_readeracc[it][2],txt_readeracc[it][3],txt_readeracc[it][4],txt_readeracc[it][5], str(adjusted_time)]
            write = csv.writer(csv_writer)
            write.writerow(extr)
    with open('gyronew.txt','w+',newline='') as csv_writer:
        for it in range(len(txt_readergyr)):    
            time = int(txt_readergyr[it][6])
            adjusted_time = int(( time*100 )-( CONST_EPOCH ))
            extr = [txt_readergyr[it][1],txt_readergyr[it][2],txt_readergyr[it][3],txt_readergyr[it][4],txt_readergyr[it][5],str(adjusted_time)]
            write = csv.writer(csv_writer)
            write.writerow(extr)


def extract():
    accelData = np.loadtxt('accelnew.txt', dtype = np.float64, delimiter=',',usecols=(0,1,2,3,4))
    gyroData  = np.loadtxt('gyronew.txt', dtype = np.float64, delimiter=',',usecols=(0,1,2,3,4))
    h_a_time  = np.loadtxt('accelnew.txt', dtype=Decimal, delimiter=',', usecols = (5))
    h_g_time  = np.loadtxt('gyronew.txt', dtype=Decimal, delimiter=',', usecols = (5))
    
    h_a_time_epoch = np.arange(len(h_a_time),dtype = Decimal)
    h_a_time_experiment = np.arange(len(h_a_time),dtype = Decimal)
    h_a_time_secs = np.arange(len(h_a_time),dtype = np.float64)
    h_a_time_nsecs = np.arange(len(h_a_time),dtype = np.float64) 
    
    h_g_time_epoch = np.arange(len(h_g_time),dtype = Decimal)
    h_g_time_experiment = np.arange(len(h_g_time),dtype = Decimal)
    h_g_time_secs = np.arange(len(h_g_time),dtype = np.float64)
    h_g_time_nsecs = np.arange(len(h_g_time),dtype = np.float64) 
    
    decs_a= np.arange(len(h_a_time),dtype = Decimal)
    decns_a= np.arange(len(h_a_time),dtype = Decimal)

    decs_g= np.arange(len(h_g_time),dtype = Decimal)
    decns_g= np.arange(len(h_g_time),dtype = Decimal)
    
    for i in range(len(h_a_time)):
        tempStr = str(h_a_time[i])
        h_a_time_secs[i]  = float(tempStr[0:10])
        h_a_time_nsecs[i] = float(tempStr[10:19])
        
        decs_a[i] =Decimal(h_a_time_secs[i])
        decns_a[i] = Decimal(h_a_time_nsecs[i])
        
        h_a_time_epoch[i] = decs_a[i] + decns_a[i]*Decimal(1e-9)
        tempStr = str(h_a_time_epoch[i])
        tempStr = tempStr[0:20]
        h_a_time_epoch[i] = Decimal(tempStr)
        h_a_time_experiment[i] = h_a_time_epoch[i] - h_a_time_epoch[0]


    
    for i in range(len(h_g_time)):    
        tempStr = str(h_g_time[i])
        h_g_time_secs[i]  = float(tempStr[0:10])
        h_g_time_nsecs[i] = float(tempStr[10:19])
        
        decs_g[i] =Decimal(h_g_time_secs[i])
        decns_g[i] = Decimal(h_g_time_nsecs[i])
        
        h_g_time_epoch[i] = decs_g[i] + decns_g[i]*Decimal(1e-9)
        tempStr = str(h_g_time_epoch[i])
        tempStr = tempStr[0:20]
        h_g_time_epoch[i] = Decimal(tempStr)
        h_g_time_experiment[i] = h_g_time_epoch[i] - h_g_time_epoch[0]    
   
   
    #grav axis is signed (-) which doesnt fit with the ENU frame
    # threfore manually corrected here
    hlAccX          = -accelData[:,0]
    hlAccY          = accelData[:,1]
    hlAccZ          = accelData[:,2]
    hlAccNorm       = accelData[:,3]
    hlAccSoCtime    = accelData[:,4]
    
    hlGyrX          = gyroData[:,0]
    hlGyrY          = gyroData[:,1]
    hlGyrZ          = gyroData[:,2]
    hlGyrNorm       = gyroData[:,3]
    hlGyrSoCtime    = gyroData[:,4]
    
    return (hlAccX, hlAccY, hlAccZ,
            hlAccNorm, hlGyrX, hlGyrY,
            hlGyrZ, hlGyrNorm, h_a_time_epoch,
            h_a_time_experiment, h_g_time_epoch, h_g_time_experiment)

