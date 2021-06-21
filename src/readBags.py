import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
import re
from decimal import Decimal
import csv
import matplotlib.pyplot as plt
import os, sys, time, pathlib
import math

def parseXsensBag():
    '''
    this parses both the regular topics and also uses regular expressions
    to parse the whole data string coming from the raw topic.
    the length of raw data is changing all the time so i have some long if else loops to account for all the
    sensor readings

    '''
    bag = bagreader('output.bag')
    
    imuData       = bag.message_by_topic('/imu/data')
    imuMag        = bag.message_by_topic('/imu/mag')
    imuDataStr    = bag.message_by_topic('/imu_data_str')
    
    csvImuData    = pd.read_csv(imuData)
    csvImuMag     = pd.read_csv(imuMag)
    csvImuDataStr = pd.read_csv(imuDataStr)
    
    
    # get /imu/data readings
   # rosTime = csvImuData['Time'].to_numpy()
    rosTimeSecs = csvImuData['header.stamp.secs'].to_numpy()
    rosTimeNsecs = csvImuData['header.stamp.nsecs'].to_numpy()
    
    rosTimeSecs = np.float64(rosTimeSecs)
    rosTimeNsecs = np.float64(rosTimeNsecs)
    decimal_s  = np.arange(len(rosTimeSecs),dtype = Decimal)
    decimal_ns = np.arange(len(rosTimeSecs),dtype = Decimal)
    decimal_conc = np.arange(len(rosTimeSecs),dtype = Decimal)
    rosTime= np.arange(len(rosTimeSecs),dtype = Decimal)        
        
        

    for i in range(len(rosTimeSecs)):
        decimal_s[i] = Decimal(rosTimeSecs[i])
        decimal_ns[i] = Decimal(rosTimeNsecs[i])
        
        decimal_conc[i] = decimal_s[i] + decimal_ns[i]*Decimal(1e-9)
        tempStr = str(decimal_conc[i])
        tempStr = tempStr[0:20]
        rosTime[i] = Decimal(tempStr)
    

    
    orientx = csvImuData['orientation.x'].to_numpy()
    orienty = csvImuData['orientation.y'].to_numpy()
    orientz = csvImuData['orientation.z'].to_numpy()
    orientw = csvImuData['orientation.w'].to_numpy()
    
    angVelx = csvImuData['angular_velocity.x'].to_numpy()
    angVely = csvImuData['angular_velocity.y'].to_numpy()
    angVelz = csvImuData['angular_velocity.z'].to_numpy()
    
    linAccx = csvImuData['linear_acceleration.x'].to_numpy()
    linAccy = csvImuData['linear_acceleration.y'].to_numpy()
    linAccz = csvImuData['linear_acceleration.z'].to_numpy()
    
    # get /imu/mag readings
    magx    =csvImuMag['magnetic_field.x'].to_numpy()
    magy    =csvImuMag['magnetic_field.y'].to_numpy()
    magz    =csvImuMag['magnetic_field.z'].to_numpy()
    
    # get /imu_data_str unparsed message readings
    allString = csvImuDataStr['data'].tolist()
    
    hour = re.compile(r"'Hour':\s\d")
    minute = re.compile(r"'Minute':\s\d*")
    second = re.compile(r"'Second':\s\d*")
    ns = re.compile(r"'ns': \d*")
    
    magX   = re.compile(r"'magX': ...\d*")
    magY   = re.compile(r"'magY': ...\d*")
    magZ   = re.compile(r"'magZ': ...\d*")
    
    quat  = re.compile(r"'quaternion': \(.*\)") 
    groupmatch = re.compile(r'\d\.\d*')
    
    gyroX = re.compile(r"'gyrX': [+-]?((\d+\.\d*)|(\.\d+)|(\d+))([eE][+-]?\d+)?")
    gyroY = re.compile(r"'gyrY': [+-]?((\d+\.\d*)|(\.\d+)|(\d+))([eE][+-]?\d+)?")
    gyroZ = re.compile(r"'gyrZ': [+-]?((\d+\.\d*)|(\.\d+)|(\d+))([eE][+-]?\d+)?")
    accelX = re.compile(r"'accX': [+-]?((\d+\.\d*)|(\.\d+)|(\d+))([eE][+-]?\d+)?")
    accelY = re.compile(r"'accY': [+-]?((\d+\.\d*)|(\.\d+)|(\d+))([eE][+-]?\d+)?")
    accelZ = re.compile(r"'accZ': [+-]?((\d+\.\d*)|(\.\d+)|(\d+))([eE][+-]?\d+)?")
    linvelxpattern = re.compile(r"'Vel_X': [+-]?((\d+\.\d*)|(\.\d+)|(\d+))([eE][+-]?\d+)?")
    linvelypattern = re.compile(r"'Vel_Y': [+-]?((\d+\.\d*)|(\.\d+)|(\d+))([eE][+-]?\d+)?")
    linvelzpattern = re.compile(r"'Vel_Z': [+-]?((\d+\.\d*)|(\.\d+)|(\d+))([eE][+-]?\d+)?")

    
    
    
    utcHours    = np.arange(len(allString), dtype = np.int64)
    utcSecs     = np.arange(len(allString), dtype = np.int64)
    utcNsecs    = np.arange(len(allString), dtype = np.int64)
    utcMins     = np.arange(len(allString), dtype = np.int64)
    accX        = np.arange(len(allString), dtype = np.float64)
    accY        = np.arange(len(allString), dtype = np.float64)
    accZ        = np.arange(len(allString), dtype = np.float64)
    gyrX        = np.arange(len(allString), dtype = np.float64)
    gyrY        = np.arange(len(allString), dtype = np.float64)
    gyrZ        = np.arange(len(allString), dtype = np.float64)
    magnoX      = np.arange(len(allString), dtype = np.float64)
    magnoY      = np.arange(len(allString), dtype = np.float64)
    magnoZ      = np.arange(len(allString), dtype = np.float64)
    quatW       = np.arange(len(allString), dtype = np.float64)
    quatX       = np.arange(len(allString), dtype = np.float64)
    quatY       = np.arange(len(allString), dtype = np.float64)
    quatZ       = np.arange(len(allString), dtype = np.float64)
    
    linvelX     = np.arange(len(allString), dtype = np.float64)
    linvelY     = np.arange(len(allString), dtype = np.float64)
    linvelZ     = np.arange(len(allString), dtype = np.float64)

    
    #parse the whole data string
    
    #parse UTC time
    for i in range(len(allString)):
        
        hourSearch = re.search(hour,allString[i])
        temp = hourSearch.group(0)
        if len(temp) == 9:
            utcHours[i] = int(temp[8:9])
        elif len(temp) == 10:
            utcHours[i] = int(temp[8:10])
    
        minuteSearch = re.search(minute,allString[i])
        temp = minuteSearch.group(0)
        if len(temp) == 11:
            utcMins[i] = int(temp[10:11])
        elif len(temp) == 12:
            utcMins[i] = int(temp[10:12])
            
        secondSearch = re.search(second,allString[i])
        temp = secondSearch.group(0)
        if len(temp) == 11:
            utcSecs[i] = int(temp[10:11])
        elif len(temp) ==12:
            utcSecs[i] = int(temp[10:12])
            
        nsecSearch = re.search(ns,allString[i])
        temp = nsecSearch.group(0)  
        if len(temp) == 7:
            utcNsecs[i] = int(temp[6:7])
        elif len(temp) ==13:
            utcNsecs[i] = int(temp[6:13])
        elif len(temp) ==14:
            utcNsecs[i] = int(temp[6:14])
        elif len(temp) ==15:
            utcNsecs[i] = int(temp[6:15])
    
    utcTime = np.arange(len(allString), dtype= np.float64)
    for i in range(len(allString)):
        utcTime[i] = ( (float(60 * utcMins[i] * 1e9) + float(utcSecs[i]*1e9) + utcNsecs[i])
                      -
                     (float(60 * utcMins[0] * 1e9) + float(utcSecs[0]*1e9) + utcNsecs[0]) ) 
    
        utcTime[i] = utcTime[i]*1e-9
        tempStr = str(utcTime[i])
        tempStr = tempStr[0:15]
        utcTime[i] = tempStr
    
    #parse quaternions
        quatsearch    = re.search(quat,allString[i])
        quatgroup     = quatsearch.group(0)
        allquats      = re.findall(groupmatch,quatgroup)
        quatW[i]      = float(allquats[0])
        quatX[i]      = float(allquats[1])
        quatY[i]      = float(allquats[2])
        quatZ[i]      = float(allquats[3])
        
    #parse accels
        accSearchX = re.search(accelX,allString[i])
        tempX      = accSearchX.group(0)
        accSearchY = re.search(accelY,allString[i])
        tempY      = accSearchY.group(0)
        accSearchZ = re.search(accelZ,allString[i])
        tempZ      = accSearchZ.group(0)
        
        
        if len(tempX) == 13:
            accX[i] = float(tempX[8:13])          
        elif len(tempX) == 14:
            accX[i] = float(tempX[8:14])              
        elif len(tempX) == 15:
            accX[i] = float(tempX[8:15])              
        elif len(tempX) == 16:
            accX[i] = float(tempX[8:16])        
        elif len(tempX) == 17:
            accX[i] = float(tempX[8:17])        
        elif len(tempX) == 18:
            accX[i] = float(tempX[8:18])        
        elif len(tempX) == 19:
            accX[i] = float(tempX[8:19])  
        elif len(tempX) == 20:
            accX[i] = float(tempX[8:20])  
        elif len(tempX) == 21:
            accX[i] = float(tempX[8:21])       
        elif len(tempX) == 22:
            accX[i] = float(tempX[8:22])       
        elif len(tempX) == 23:
            accX[i] = float(tempX[8:23])    
        elif len(tempX) == 24:
            accX[i] = float(tempX[8:24])     
        elif len(tempX) == 25:
            accX[i] = float(tempX[8:25]) 
        elif len(tempX) == 26:
            accX[i] = float(tempX[8:26]) 
        elif len(tempX) ==27:
            accX[i] = float(tempX[8:27]) 
        elif len(tempX) ==28:
            accX[i] = float(tempX[8:28]) 
        elif len(tempX) ==29:
            accX[i] = float(tempX[8:29])         
        elif len(tempX) ==30:
            accX[i] = float(tempX[8:30])         
        elif len(tempX) ==31:
            accX[i] = float(tempX[8:31]) 
        elif len(tempX) ==32:
            accX[i] = float(tempX[8:32])  
        elif len(tempX) ==33:
            accX[i] = float(tempX[8:33])  
            
        if len(tempY) == 13:
            accY[i] = float(tempY[8:13])          
        elif len(tempY) == 14:
            accY[i] = float(tempY[8:14])              
        elif len(tempY) == 15:
            accY[i] = float(tempY[8:15])   
        elif len(tempY) == 16:
            accY[i] = float(tempY[8:16])        
        elif len(tempY) == 17:
            accY[i] = float(tempY[8:17])        
        elif len(tempY) == 18:
            accY[i] = float(tempY[8:18])         
        elif len(tempY) == 19:
            accY[i] = float(tempY[8:19])  
        elif len(tempY) == 20:
            accY[i] = float(tempY[8:20])               
        elif len(tempY) == 21:
            accY[i] = float(tempY[8:21])       
        elif len(tempY) == 22:
            accY[i] = float(tempY[8:22])       
        elif len(tempY) == 23:
            accY[i] = float(tempY[8:23])    
        elif len(tempY) == 24:
            accY[i] = float(tempY[8:24])     
        elif len(tempY) == 25:
            accY[i] = float(tempY[8:25]) 
        elif len(tempY) == 26:
            accY[i] = float(tempY[8:26]) 
        elif len(tempY) ==27:
            accY[i] = float(tempY[8:27]) 
        elif len(tempY) ==28:
            accY[i] = float(tempY[8:28]) 
        elif len(tempY) ==29:
            accY[i] = float(tempY[8:29]) 
        elif len(tempY) ==30:
            accY[i] = float(tempY[8:30]) 
        elif len(tempY) ==31:
            accY[i] = float(tempY[8:31]) 
        elif len(tempY) ==32:
            accY[i] = float(tempY[8:32])  
        elif len(tempY) ==33:
            accY[i] = float(tempY[8:33])  

        if len(tempZ) == 13:
            accZ[i] = float(tempZ[8:13])          
        elif len(tempZ) == 14:
            accZ[i] = float(tempZ[8:14])              
        elif len(tempZ) == 15:
            accZ[i] = float(tempZ[8:15])       
        elif len(tempZ) == 16:
            accZ[i] = float(tempZ[8:16])        
        elif len(tempZ) == 17:
            accZ[i] = float(tempZ[8:17])        
        elif len(tempZ) == 18:
            accZ[i] = float(tempZ[8:18]) 
        elif len(tempZ) == 19:
            accZ[i] = float(tempZ[8:19])  
        elif len(tempZ) == 20:
            accZ[i] = float(tempZ[8:20]) 
        elif len(tempZ) == 21:
            accZ[i] = float(tempZ[8:21])       
        elif len(tempZ) == 22:
            accZ[i] = float(tempZ[8:22])       
        elif len(tempZ) == 23:
            accZ[i] = float(tempZ[8:23])    
        elif len(tempZ) == 24:
            accZ[i] = float(tempZ[8:24])     
        elif len(tempZ) == 25:
            accZ[i] = float(tempZ[8:25]) 
        elif len(tempZ) == 26:
            accZ[i] = float(tempZ[8:26]) 
        elif len(tempZ) ==27:
            accZ[i] = float(tempZ[8:27]) 
        elif len(tempZ) ==28:
            accZ[i] = float(tempZ[8:28])
        elif len(tempZ) ==29:
            accZ[i] = float(tempZ[8:29]) 
        elif len(tempZ) ==30:
            accZ[i] = float(tempZ[8:30])
        elif len(tempZ) ==31:
            accZ[i] = float(tempZ[8:31]) 
        elif len(tempZ) ==32:
            accZ[i] = float(tempZ[8:32])  
        elif len(tempZ) ==33:
            accZ[i] = float(tempZ[8:33])  
    
    #parse gyroscope
        gyrSearchX = re.search(gyroX,allString[i])
        tempX      = gyrSearchX.group(0)
        gyrSearchY = re.search(gyroY,allString[i])
        tempY      = gyrSearchY.group(0)
        gyrSearchZ = re.search(gyroZ,allString[i])
        tempZ      = gyrSearchZ.group(0)

        if len(tempX) == 13:
            gyrX[i] = float(tempX[8:13])          
        elif len(tempX) == 14:
            gyrX[i] = float(tempX[8:14])              
        elif len(tempX) == 15:
            gyrX[i] = float(tempX[8:15])   
        elif len(tempX) == 16:
            gyrX[i] = float(tempX[8:16])        
        elif len(tempX) == 17:
            gyrX[i] = float(tempX[8:17])        
        elif len(tempX) == 18:
            gyrX[i] = float(tempX[8:18]) 
        elif len(tempX) == 19:
            gyrX[i] = float(tempX[8:19])  
        elif len(tempX) == 20:
            gyrX[i] = float(tempX[8:20])        
        elif len(tempX) == 21:
            gyrX[i] = float(tempX[8:21])       
        elif len(tempX) == 22:
            gyrX[i] = float(tempX[8:22])       
        elif len(tempX) == 23:
            gyrX[i] = float(tempX[8:23])    
        elif len(tempX) == 24:
            gyrX[i] = float(tempX[8:24])     
        elif len(tempX) == 25:
            gyrX[i] = float(tempX[8:25]) 
        elif len(tempX) == 26:
            gyrX[i] = float(tempX[8:26]) 
        elif len(tempX) ==27:
            gyrX[i] = float(tempX[8:27]) 
        elif len(tempX) ==28:
            gyrX[i] = float(tempX[8:28]) 
        elif len(tempX) ==29:
            gyrX[i] = float(tempX[8:29])         
        elif len(tempX) ==30:
            gyrX[i] = float(tempX[8:30])       
        elif len(tempX) ==31:
            gyrX[i] = float(tempX[8:31]) 
        elif len(tempX) ==32:
            gyrX[i] = float(tempX[8:32])  
        elif len(tempX) ==33:
            gyrX[i] = float(tempX[8:33])  

        if len(tempY) == 13:
            gyrY[i] = float(tempY[8:13])          
        elif len(tempY) == 14:
            gyrY[i] = float(tempY[8:14])              
        elif len(tempY) == 15:
            gyrY[i] = float(tempY[8:15])  
        if len(tempY) == 16:
            gyrY[i] = float(tempY[8:16])        
        elif len(tempY) == 17:
            gyrY[i] = float(tempY[8:17])        
        elif len(tempY) == 18:
            gyrY[i] = float(tempY[8:18]) 
        elif len(tempY) == 19:
            gyrY[i] = float(tempY[8:19])  
        elif len(tempY) == 20:
            gyrY[i] = float(tempY[8:20])        
        elif len(tempY) == 21:
            gyrY[i] = float(tempY[8:21])       
        elif len(tempY) == 22:
            gyrY[i] = float(tempY[8:22])       
        elif len(tempY) == 23:
            gyrY[i] = float(tempY[8:23])    
        elif len(tempY) == 24:
            gyrY[i] = float(tempY[8:24])     
        elif len(tempY) == 25:
            gyrY[i] = float(tempY[8:25]) 
        elif len(tempY) == 26:
            gyrY[i] = float(tempY[8:26]) 
        elif len(tempY) ==27:
            gyrY[i] = float(tempY[8:27]) 
        elif len(tempY) ==28:
            gyrY[i] = float(tempY[8:28]) 
        elif len(tempY) ==29:
            gyrY[i] = float(tempY[8:29]) 
        elif len(tempY) ==30:
            gyrY[i] = float(tempY[8:30]) 
        elif len(tempY) ==31:
            gyrY[i] = float(tempY[8:31]) 
        elif len(tempY) ==32:
            gyrY[i] = float(tempY[8:32])  
        elif len(tempY) ==33:
            gyrY[i] = float(tempY[8:33])  

        if len(tempZ) == 13:
            gyrZ[i] = float(tempZ[8:13])          
        elif len(tempZ) == 14:
            gyrZ[i] = float(tempZ[8:14])              
        elif len(tempZ) == 15:
            gyrZ[i] = float(tempZ[8:15])  
        elif len(tempZ) == 16:
            gyrZ[i] = float(tempZ[8:16])        
        elif len(tempZ) == 17:
            gyrZ[i] = float(tempZ[8:17])        
        elif len(tempZ) == 18:
            gyrZ[i] = float(tempZ[8:18]) 
        elif len(tempZ) == 19:
            gyrZ[i] = float(tempZ[8:19])  
        elif len(tempZ) == 20:
            gyrZ[i] = float(tempZ[8:20])     
        elif len(tempZ) == 21:
            gyrZ[i] = float(tempZ[8:21])       
        elif len(tempZ) == 22:
            gyrZ[i] = float(tempZ[8:22])       
        elif len(tempZ) == 23:
            gyrZ[i] = float(tempZ[8:23])    
        elif len(tempZ) == 24:
            gyrZ[i] = float(tempZ[8:24])     
        elif len(tempZ) == 25:
            gyrZ[i] = float(tempZ[8:25]) 
        elif len(tempZ) == 26:
            gyrZ[i] = float(tempZ[8:26]) 
        elif len(tempZ) ==27:
            gyrZ[i] = float(tempZ[8:27]) 
        elif len(tempZ) ==28:
            gyrZ[i] = float(tempZ[8:28])
        elif len(tempZ) ==29:
            gyrZ[i] = float(tempZ[8:29]) 
        elif len(tempZ) ==30:
            gyrZ[i] = float(tempZ[8:30])
        elif len(tempZ) ==31:
            gyrZ[i] = float(tempZ[8:31]) 
        elif len(tempZ) ==32:
            gyrZ[i] = float(tempZ[8:32])  
        elif len(tempZ) ==33:
            gyrZ[i] = float(tempZ[8:33])
            
            #parse linear velocities
        velSearchX = re.search(linvelxpattern,allString[i])
        tempX      = velSearchX.group(0)
        velSearchY = re.search(linvelypattern,allString[i])
        tempY      = velSearchY.group(0)
        velSearchZ = re.search(linvelzpattern,allString[i])
        tempZ      = velSearchZ.group(0)

        if len(tempX) == 16:
            linvelX[i] = float(tempX[9:16])        
        elif len(tempX) == 17:
            linvelX[i] = float(tempX[9:17])        
        elif len(tempX) == 18:
            linvelX[i] = float(tempX[9:18]) 
        elif len(tempX) == 19:
            linvelX[i] = float(tempX[9:19])  
        elif len(tempX) == 20:
            linvelX[i] = float(tempX[9:20])        
        elif len(tempX) == 21:
            linvelX[i] = float(tempX[9:21])       
        elif len(tempX) == 22:
            linvelX[i] = float(tempX[9:22])       
        elif len(tempX) == 23:
            linvelX[i] = float(tempX[9:23])    
        elif len(tempX) == 24:
            linvelX[i] = float(tempX[9:24])     
        elif len(tempX) == 25:
            linvelX[i] = float(tempX[9:25]) 
        elif len(tempX) == 26:
            linvelX[i] = float(tempX[9:26]) 
        elif len(tempX) ==27:
            linvelX[i] = float(tempX[9:27]) 
        elif len(tempX) ==28:
            linvelX[i] = float(tempX[9:28]) 
        elif len(tempX) ==29:
            linvelX[i] = float(tempX[9:29])         
        elif len(tempX) ==30:
            linvelX[i] = float(tempX[9:30])       
        elif len(tempX) ==31:
            linvelX[i] = float(tempX[9:31]) 
        elif len(tempX) ==32:
            linvelX[i] = float(tempX[9:32])  
        elif len(tempX) ==33:
            linvelX[i] = float(tempX[9:33])  

        if len(tempY) == 16:
            linvelY[i] = float(tempY[9:16])        
        elif len(tempY) == 17:
            linvelY[i] = float(tempY[9:17])        
        elif len(tempY) == 18:
            linvelY[i] = float(tempY[9:18]) 
        elif len(tempY) == 19:
            linvelY[i] = float(tempY[9:19])  
        elif len(tempY) == 20:
            linvelY[i] = float(tempY[9:20])        
        elif len(tempY) == 21:
            linvelY[i] = float(tempY[9:21])       
        elif len(tempY) == 22:
            linvelY[i] = float(tempY[9:22])       
        elif len(tempY) == 23:
            linvelY[i] = float(tempY[9:23])    
        elif len(tempY) == 24:
            linvelY[i] = float(tempY[9:24])     
        elif len(tempY) == 25:
            linvelY[i] = float(tempY[9:25]) 
        elif len(tempY) == 26:
            linvelY[i] = float(tempY[9:26]) 
        elif len(tempY) ==27:
            linvelY[i] = float(tempY[9:27]) 
        elif len(tempY) ==28:
            linvelY[i] = float(tempY[9:28]) 
        elif len(tempY) ==29:
            linvelY[i] = float(tempY[9:29]) 
        elif len(tempY) ==30:
            linvelY[i] = float(tempY[9:30]) 
        elif len(tempY) ==31:
            linvelY[i] = float(tempY[9:31]) 
        elif len(tempY) ==32:
            linvelY[i] = float(tempY[9:32])  
        elif len(tempY) ==33:
            linvelY[i] = float(tempY[9:33])  

        if len(tempZ) == 16:
            linvelZ[i] = float(tempZ[9:16])        
        elif len(tempZ) == 17:
            linvelZ[i] = float(tempZ[9:17])        
        elif len(tempZ) == 18:
            linvelZ[i] = float(tempZ[9:18]) 
        elif len(tempZ) == 19:
            linvelZ[i] = float(tempZ[9:19])  
        elif len(tempZ) == 20:
            linvelZ[i] = float(tempZ[9:20])     
        elif len(tempZ) == 21:
            linvelZ[i] = float(tempZ[9:21])       
        elif len(tempZ) == 22:
            linvelZ[i] = float(tempZ[9:22])       
        elif len(tempZ) == 23:
            linvelZ[i] = float(tempZ[9:23])    
        elif len(tempZ) == 24:
            linvelZ[i] = float(tempZ[9:24])     
        elif len(tempZ) == 25:
            linvelZ[i] = float(tempZ[9:25]) 
        elif len(tempZ) == 26:
            linvelZ[i] = float(tempZ[9:26]) 
        elif len(tempZ) ==27:
            linvelZ[i] = float(tempZ[9:27]) 
        elif len(tempZ) ==28:
            linvelZ[i] = float(tempZ[9:28])
        elif len(tempZ) ==29:
            linvelZ[i] = float(tempZ[9:29]) 
        elif len(tempZ) ==30:
            linvelZ[i] = float(tempZ[9:30])
        elif len(tempZ) ==31:
            linvelZ[i] = float(tempZ[9:31]) 
        elif len(tempZ) ==32:
            linvelZ[i] = float(tempZ[9:32])  
        elif len(tempZ) ==33:
            linvelZ[i] = float(tempZ[9:33])
        #compensate for gravity sign
        #accX = -accX    
    return (accX, accY, accZ,
            gyrX, gyrY, gyrZ,
            quatW, quatX, quatY, quatZ,
            linvelX, linvelY, linvelZ,
            utcTime, rosTime)
                