import matplotlib.pyplot as plt
import numpy as np
import math
from decimal import Decimal
from scipy.interpolate import interp1d,CubicSpline
from scipy import signal


def interpolate(accx, accy, accz,
               gyrx, gyry, gyrz,
               h_a_time_experiment_aug, h_g_time_experiment_aug,
               ):
    '''
    linear interpolation so that the erratic samples of the HL are uniform
    
    '''
    
    #timestamps at which we want to estimate the interpolated sensor readings  
    queryPointsAcc = np.linspace(float(h_a_time_experiment_aug[0]),
                                 float(h_a_time_experiment_aug[-1]),
                                 len(h_a_time_experiment_aug), endpoint=True)
    
    queryPointsGyr = np.linspace(float(h_g_time_experiment_aug[0]),
                                 float(h_g_time_experiment_aug[-1]),
                                 len(h_g_time_experiment_aug), endpoint=True)


    interp_hlGyrX = np.interp(queryPointsGyr,h_g_time_experiment_aug,gyrx)
    interp_hlGyrY = np.interp(queryPointsGyr,h_g_time_experiment_aug,gyry)
    interp_hlGyrZ = np.interp(queryPointsGyr,h_g_time_experiment_aug,gyrz)
 
    interp_hlAccX = np.interp(queryPointsAcc,h_a_time_experiment_aug,accx)
    interp_hlAccY = np.interp(queryPointsAcc,h_a_time_experiment_aug,accy)
    interp_hlAccZ = np.interp(queryPointsAcc,h_a_time_experiment_aug,accz)


    return (queryPointsAcc,queryPointsGyr,
            interp_hlAccX, interp_hlAccY, interp_hlAccZ,
            interp_hlGyrX, interp_hlGyrY, interp_hlGyrZ,
            )
