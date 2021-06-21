import numpy as np
import bagpy
from bagpy import bagreader
from scipy import signal
from decimal import Decimal


def writeMatrices(coarseAligned_xTime, rosTime, coarseAligned_qpG, initDiff,
              coarseAligned_hlGyrX, coarseAligned_hlGyrY, coarseAligned_hlGyrZ,
              coarseAligned_hlAccX, coarseAligned_hlAccY, coarseAligned_hlAccZ,
              coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ, 
              coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ):
    
    '''
    create formatted matrix with all the data inside so that it can be easily written to rosbags later on
    '''
    #adjusted_xsenstime = np.arange(len(coarseAligned_xTime),dtype=)
    #adjusted_xsenstime = coarseAligned_xTime + float(rosTime[0])
    #adjusted_hltime    = float(rosTime[0]) +coarseAligned_qpG
     
    adjusted_xsenstime = [[0 for x in range(1)] for x in range(len(coarseAligned_xTime))]
    adjusted_hltime = [[0 for x in range(1)] for x in range(len(coarseAligned_qpG))]

        
    for i in range(len(coarseAligned_xTime)):
        temp_decimal = coarseAligned_xTime[i]
        temp_decimal = str(temp_decimal)
        temp_decimal = temp_decimal[0:9]
        adjusted_xsenstime[i] = Decimal(temp_decimal) + rosTime[0]
        tempstr    =  str(adjusted_xsenstime[i])
        tempstr = tempstr[0:20]
        adjusted_xsenstime[i] = tempstr

    for i in range(len(coarseAligned_qpG)):
        temp_decimal = coarseAligned_qpG[i]
        temp_decimal = str(temp_decimal)
        temp_decimal = temp_decimal[0:9]
        adjusted_hltime[i] = Decimal(temp_decimal) + rosTime[0] + Decimal(5)
        tempstr    =  str(adjusted_hltime[i])
        tempstr = tempstr[0:20]
        adjusted_hltime[i] = tempstr
        
        
    #format HL
    coarseAligned_hlAccX = np.transpose(coarseAligned_hlGyrX)
    coarseAligned_hlAccY = np.transpose(coarseAligned_hlGyrY)
    coarseAligned_hlAccZ = np.transpose(coarseAligned_hlGyrZ)
    coarseAligned_hlGyrX = np.transpose(coarseAligned_hlGyrX)
    coarseAligned_hlGyrY = np.transpose(coarseAligned_hlGyrY)
    coarseAligned_hlGyrZ = np.transpose(coarseAligned_hlGyrZ)
    #write the matrix in NED (north east down) frame

    hlMatrix = np.array([
            [coarseAligned_hlAccZ],
            [-coarseAligned_hlAccY],
            [coarseAligned_hlAccX],
            [coarseAligned_hlGyrZ],
            [-coarseAligned_hlGyrY],
            [coarseAligned_hlGyrX],
            [adjusted_hltime]
        ])
    hlMatrix = np.transpose(hlMatrix)
    hlMatrix = np.squeeze(hlMatrix)    
    
    #format XSENS
    accX = np.transpose(coarseAligned_xsensAccX)
    accY = np.transpose(coarseAligned_xsensAccY)
    accZ = np.transpose(coarseAligned_xsensAccZ)
    gyrX = np.transpose(coarseAligned_xsensGyrX)
    gyrY = np.transpose(coarseAligned_xsensGyrY)
    gyrZ = np.transpose(coarseAligned_xsensGyrZ)
    #write the matrix in NED (north east down) frame

    xsensMatrix = np.array([
                            [accZ],
                            [-accY],
                            [accX],
                            [gyrZ],
                            [-gyrY],
                            [gyrX],
                            [adjusted_xsenstime]
                                ])
    
    xsensMatrix = np.transpose(xsensMatrix)
    xsensMatrix = np.squeeze(xsensMatrix)    

    np.savetxt('hlmatrix.txt',hlMatrix,fmt='%s',delimiter=' ')
    np.savetxt('xsensmatrix.txt',xsensMatrix,fmt='%s',delimiter=' ')


def camAlignment(initCamDiff, rosTime, pvStamps_experiment, erasedSamplesCam):
        #cam timestamps
    '''
    cam time aligned properly with precisions preserved.
    '''
    pvStamps_experiment= np.array(list(map(np.float64,pvStamps_experiment)))

    adjusted_camtime = [[0 for x in range(1)] for x in range(len(pvStamps_experiment))]
    for i in range(len(pvStamps_experiment)):
        temp_decimal = pvStamps_experiment[i]
        temp_decimal = str(temp_decimal)
        temp_decimal = temp_decimal[0:9]
        adjusted_camtime[i] = Decimal(temp_decimal) + rosTime[0]
        tempstr    =  str(adjusted_camtime[i])
        tempstr = tempstr[0:20]
        adjusted_camtime[i] = tempstr
   # pvStamps_experiment= np.array(list(map(np.float64,pvStamps_experiment)))
    #adjusted_camtime    = float(rosTime[0])-initCamDiff +pvStamps_experiment
    #adjusted_camtime    = adjusted_camtime[erasedSamplesCam:]
    return (adjusted_camtime)