import numpy as np
import matplotlib.pyplot as plt
from DataConverter import DataConverter
from scipy.io import savemat

#init converter object
dat_conv = DataConverter()

#load the results obtained from the hexapod
res = dat_conv.load_position_file("hexameas.txt")

#query the properties of the results
shapeOfResults = res.shape
recordedTime   = res[0]
timeStamps     = res[1]
posX           = res[2]
posY           = res[3]
posZ           = res[4]
roll           = res[5]
pitch          = res[6]
yaw            = res[7]
vx             = res[8]
vy             = res[9]
vz             = res[10]
r_dot             = res[11]
p_dot             = res[12]
y_dot             = res[13]
ax             = res[14]
ay             = res[15]
az             = res[16]

#convert the euler angles to angular rates of the body frame
wbx,wby,wbz = dat_conv.rpy_dot_to_wb_dot(roll, pitch, yaw, r_dot, p_dot, y_dot)
hexIMU = np.loadtxt('hexaimu.txt', skiprows=1)

hexEpoch = hexIMU[:,0]
pyTime = hexIMU[:,1]
hexaccX = hexIMU[:,2]
hexaccY = hexIMU[:,3]
hexaccZ = hexIMU[:,4]
hexgyrX = hexIMU[:,5]
hexgyrY = hexIMU[:,6]
hexgyrZ = hexIMU[:,7]




pyTime = pyTime[:-1]
times, hexvx = dat_conv.get_time_derivative(pyTime,hexaccX)
hexvx = hexvx.transpose()
hexvx = hexvx.squeeze()

times, hexvy = dat_conv.get_time_derivative(pyTime,hexaccY)
hexvy = hexvx.transpose()
hexvy = hexvy.squeeze()

times, hexvz = dat_conv.get_time_derivative(pyTime,hexaccZ)
hexvz = hexvx.transpose()
hexvz = hexvz.squeeze()

times, hexpx = dat_conv.get_time_derivative(pyTime,hexvx)
times, hexpy = dat_conv.get_time_derivative(pyTime,hexvy)
times, hexpz = dat_conv.get_time_derivative(pyTime,hexvz)

#save to mat files
mdic = {"posX": posX,
        "posY":posY,
        "posZ":posZ,
        "roll":roll,
        "pitch":pitch,
        "yaw":yaw,
        "vx":vx,
        "vy":vy,
        "vz":vz,
        "ax":ax,
        "ay":ay,
        "az":az,
        "r_dot":r_dot,
        "p_dot":p_dot,
        "y_dot":y_dot,
        "recordedTime": recordedTime,
        "timeStamps": timeStamps,
        "wbx": wbx,
        "wby":wby,
        "wbz":wbz,
        "hexEpoch": hexEpoch,
        "pyTime" :pyTime, 
        "hexaccX": hexaccX,
        "hexaccY": hexaccY,
        "hexaccZ": hexaccZ,
        "hexgyrX": hexgyrX,
        "hexgyrY": hexgyrY,
        "hexgyrZ": hexgyrZ,
        "hexvx" : hexvx,
        "hexvy" : hexvy,
        "hexvz" : hexvz,
        "hexpx" : hexpx,
        "hexpy" : hexpy,
        "hexpz" : hexpz
        }



savemat("hexapod.mat", mdic)