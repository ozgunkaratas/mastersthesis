import os
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import RotationSpline
import math
from decimal import Decimal
from scipy.interpolate import interp1d,CubicSpline
from scipy import signal
from scipy.io import savemat



CONST_EPOCH = 11644473600000000000 #nseconds between win epoch and unix epoch

def eulerAnglesToRotMat(theta) :
    '''
    basic rotation matrices
    '''
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
        
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
    
    R = np.dot(R_z, np.dot( R_y, R_x ))
    
    return R


def rotMatToEulerAngles(R) :
    '''
    conversion between rotation matrix and euler angles
    '''
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])


def extractCameraMatrix():
    '''
    extract raw homo. matrix information from the txt file which is produced with the hl images
    '''
    dir = os.getcwd()
    if os.path.exists(dir):
        # for files in os.walk(dir):
        #     for file in files:
        #         if file[18:20] =="pv" and os.path.splitext(file)[1] == ".txt":
        #             camfile = os.path.join(dir,file)
        for path, folder, files in os.walk(dir):
            for file in files:
                if file[18:20] =="pv" and os.path.splitext(file)[1] == ".txt":
                    camfile = os.path.join(dir,file)

    with open (camfile,'r') as camreader:
        lines = camreader.readlines()

    for i in range(len(lines)):
        lines[i] = lines[i].split(",") 

    #timestamps
    camtime = np.zeros(i, dtype = np.float64)  

    #translations

    camera_posx = np.zeros(i, dtype = np.float64)
    camera_posy = np.zeros(i, dtype = np.float64)
    camera_posz = np.zeros(i, dtype = np.float64)

    # rotations
    camera_r_11 = np.zeros(i, dtype = np.float64)
    camera_r_12 = np.zeros(i, dtype = np.float64)
    camera_r_13 = np.zeros(i, dtype = np.float64)

    camera_r_21 = np.zeros(i, dtype = np.float64)
    camera_r_22 = np.zeros(i, dtype = np.float64)
    camera_r_23 = np.zeros(i, dtype = np.float64)

    camera_r_31 = np.zeros(i, dtype = np.float64)
    camera_r_32 = np.zeros(i, dtype = np.float64)
    camera_r_33 = np.zeros(i, dtype = np.float64)  

    #assign homo matrix values for all the samples
    stackedCameraMatrix = np.zeros((len(lines)-1,3,3))
    stackedCameraTrans  = np.zeros((len(lines)-1,3))
    for i in range(len(lines)):
        #both in nsecs
        camtime[i] = float(lines[i+1][0])*100 -CONST_EPOCH
        
        camera_posx[i] = lines[i+1][6]
        camera_posy[i] = lines[i+1][10]
        camera_posz[i] = lines[i+1][14]
        
        camera_r_11[i] = lines[i+1][3]
        camera_r_12[i] = lines[i+1][4]
        camera_r_13[i] = lines[i+1][5]
        
        camera_r_21[i] = lines[i+1][7]
        camera_r_22[i] = lines[i+1][8]
        camera_r_23[i] = lines[i+1][9]
        
        camera_r_31[i] = lines[i+1][11]
        camera_r_32[i] = lines[i+1][12]
        camera_r_33[i] = lines[i+1][13]
        
        temp_cr11 = camera_r_11[i]
        temp_cr12 = camera_r_12[i]
        temp_cr13 = camera_r_13[i]
        temp_cpx  = camera_posx[i]
        temp_cr21 = camera_r_21[i]
        temp_cr22 = camera_r_22[i]
        temp_cr23 = camera_r_23[i]
        temp_cpy  = camera_posy[i]
        temp_cr31 = camera_r_31[i]
        temp_cr32 = camera_r_32[i]
        temp_cr33 = camera_r_33[i]
        temp_cpz  = camera_posz[i]
        
        stackedCameraMatrix[i,:,:]= [[temp_cr11,temp_cr12,temp_cr13],
                        [temp_cr21,temp_cr22,temp_cr23],
                        [temp_cr31,temp_cr32,temp_cr33]]
        
        stackedCameraTrans[i,:] = [temp_cpx, temp_cpy, temp_cpz]
        
        if i == (len(lines)-2):
            break
    #pv jumps to unrealistic translational values (stack overflow?) , correct for this by asserting that two 
    #consequtive position values cannot move extremely fast (in this case, 1 meter per sample)
    for i in range(len(stackedCameraTrans)):

            #do x
        if stackedCameraTrans[i,0] - stackedCameraTrans[i-1,0] > 1:
            #print("hello, %f",i)
            stackedCameraTrans[i-1,0] = stackedCameraTrans[i-2,0]
            if i == len(stackedCameraTrans):
                break
            

            #do y
        if stackedCameraTrans[i,1] - stackedCameraTrans[i-1,1] > 1:
            #print("hello, %f",i)
            stackedCameraTrans[i-1,1] = stackedCameraTrans[i-2,1]
            if i == len(stackedCameraTrans):
                break
            

            #do z
        if stackedCameraTrans[i,2] - stackedCameraTrans[i-1,2] > 1:
            #print("hello, %f",i)
            stackedCameraTrans[i-1,2] = stackedCameraTrans[i-2,2]
            if i == len(stackedCameraTrans):
                break
            
            
    #rotation formalities conversions

    camRotateFromMatrix = Rotation.from_matrix(stackedCameraMatrix)
    camQuat = camRotateFromMatrix.as_quat()
    camEul = camRotateFromMatrix.as_euler('zyx', degrees = True)
    camRotVec = camRotateFromMatrix.as_rotvec()

    #euler angles from basic conversion
    eulerX = np.zeros(len(camera_posx), dtype = np.float64)
    eulerY = np.zeros(len(camera_posx), dtype = np.float64)
    eulerZ = np.zeros(len(camera_posx), dtype = np.float64)
    #in radians
    for i in range(len(camera_posx)):
        eulerX[i], eulerY[i], eulerZ[i] = rotMatToEulerAngles(stackedCameraMatrix[i,:,:])
        
    return(stackedCameraMatrix, stackedCameraTrans,
            camRotateFromMatrix, camQuat, camEul, camRotVec,camtime)

def alignCamera(camrotmat, camtransmat, erasedSamples, pvFPS,
                splineDelays,initDiff,pvStart, gyroStart,camtime):
    '''
    classic temporal alignment scheme via experiment duration checks
    '''
        #HL ran more, beginning of HL stream will be snipped
    if initDiff > 0 :
        shiftSamples = erasedSamples + int(np.ceil(pvFPS*splineDelays))
        
        alignedStackedCameraMatrix = camrotmat[shiftSamples:]
        alignedStackedCameraTrans = camtransmat[shiftSamples:]
        camtime =camtime[shiftSamples:]

        #this is the sensor start up delay (in samples)between HL IMU and HL cam
        internalShift = float(pvStart - gyroStart) / (1/pvFPS)
        internalShift = int(np.ceil(internalShift))
        
        alignedStackedCameraMatrix = alignedStackedCameraMatrix[internalShift:]
        alignedStackedCameraTrans = alignedStackedCameraTrans[internalShift:]
        camtime =camtime[internalShift:]
        
   #xsens ran more, cam doesnt need snipping, but internal shift must be calculated
    if initDiff < 0:
        #print("Camera values are left unchanged\n")
        internalShift = float(pvStart - gyroStart) / (1/pvFPS)
        internalShift = int(np.ceil(internalShift))
        
        alignedStackedCameraMatrix = camrotmat[internalShift:]
        alignedStackedCameraTrans = camtransmat[internalShift:]   
        camtime = camtime[internalShift:]
        
    return(alignedStackedCameraMatrix, alignedStackedCameraTrans, camtime)


def quatInterpolate(erasedSamplesCam,coarseAligned_xTime, pvStamps_experiment,
                    stackedCameraMatrix, stackedCameraTrans,
                    camRotateFromMatrix, camQuat, camEul, camRotVec):
    '''
    Slerp the quats which originate from rot matrix conversions
    rot matrix cant be interpolated efficiently so its better to convert
    everything to quats and slerp them.
    
    '''
    #cast stamps to float
    times = np.array(list(map(np.float64,pvStamps_experiment)))
    
    #RotationSpline
    angles = camEul
    rotations = Rotation.from_euler('XYZ',angles,degrees=True)
    rotSpline = RotationSpline(times,rotations)
    angularRate = np.rad2deg(rotSpline(times, 1))
    angularAcceleration = np.rad2deg(rotSpline(times, 2))
    timesPlot = np.linspace(times[0], times[-1], 5)
    anglesPlot = rotSpline(timesPlot).as_euler('XYZ', degrees=True)
    angularRatePlot = np.rad2deg(rotSpline(timesPlot, 1))
    angularAccelerationPlot = np.rad2deg(rotSpline(timesPlot, 2))
    
    #SLERP Quat interpolation
    rotations = Rotation.from_quat(camQuat)
    slerp = Slerp(times,rotations)
    linspaceTime = np.linspace(times[0], times[-1], num=len(coarseAligned_xTime))
    slerpedRots = slerp(linspaceTime)
    beforeInterpQuat= rotations.as_quat()
    interpQuat = slerpedRots.as_quat()
    
    #SLERP Euler interpolation
    rotations = Rotation.from_matrix(stackedCameraMatrix)
    slerp = Slerp(times,rotations)
    interpEulRots = slerp(linspaceTime)    
    beforeInterpEul= rotations.as_euler('xyz', degrees=True)
    interpEuler = interpEulRots.as_euler('xyz', degrees = True)
    
    return(angularRate,angularAcceleration,timesPlot,anglesPlot,angularRatePlot,angularAccelerationPlot,
           linspaceTime, beforeInterpQuat, interpQuat, beforeInterpEul, interpEuler)
    
    
    pass

def upsampleTranslation(cam_Matrix,xsensLength,aligncamtime):
    '''
    upsample the trans matrix to match xsens time
    '''
    NED_pv_X = -cam_Matrix[:,0]
    NED_pv_Y = -cam_Matrix[:,2]
    NED_pv_Z = -cam_Matrix[:,1]
    up_NED_pv_X, upNEDTime = signal.resample(-cam_Matrix[:,0],len(xsensLength),aligncamtime) 
    up_NED_pv_Y, upNEDTime = signal.resample(-cam_Matrix[:,2],len(xsensLength),aligncamtime) 
    up_NED_pv_Z, upNEDTime = signal.resample(-cam_Matrix[:,1],len(xsensLength),aligncamtime) 

    return(up_NED_pv_X, up_NED_pv_Y, up_NED_pv_Z, upNEDTime)


def saveMatlab(up_NED_pv_X, up_NED_pv_Y, up_NED_pv_Z, upNEDTime):
    '''
    post processing tasks
    '''

    mdic = {"up_NED_pv_X": up_NED_pv_X,
        "up_NED_pv_Y":up_NED_pv_Y,
        "up_NED_pv_Z":up_NED_pv_Z,
        "upNEDTime": upNEDTime
            }
    savemat("pvCamera.mat", mdic)
    pass
